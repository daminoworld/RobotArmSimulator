using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace RobotArmSimulator
{
    public enum PoseDataset
    {
        TaskAPlane,
        TaskBCylinder,
        TaskCCoordinateTransform
    }

    public sealed class RobotTaskLoader : MonoBehaviour
    {
        [Header("Pose Dataset")]
        [SerializeField] private PoseDataset poseDataset = PoseDataset.TaskAPlane;
        [SerializeField] private TextAsset taskAPlanePoseJson;
        [SerializeField] private TextAsset taskBCylinderPoseJson;
        [SerializeField] private TextAsset taskCCoordinatePoseJson;

        [Header("Trajectory Source")]
        [SerializeField] private TextAsset trajectoryCsv;

        [Header("Fallback Paths")]
        [SerializeField] private string taskAPlaneFallbackRelativePath = "SampleData/task_a_motion_editor_plane.json";
        [SerializeField] private string taskBCylinderFallbackRelativePath = "SampleData/task_b_motion_editor_cylinder.json";
        [SerializeField] private string taskCCoordinateFallbackRelativePath = "SampleData/task_c_coordinate_transform.json";
        [SerializeField] private string trajectoryFallbackRelativePath = "SampleData/task_d_joint_trajectory.csv";

        [Header("Options")]
        [SerializeField] private bool loadOnStart = true;
        [SerializeField] private bool logLoadedData = true;
        [Header("Coordinate Transform Alignment")]
        [SerializeField] private Vector3 taskCCoordinateWorldPoseOffset = new Vector3(-0.09f, -0.91f, -0.823f);

        public event Action<RobotTaskData> OnTaskLoaded;
        public event Action<string> OnTaskLoadFailed;

        public RobotTaskData CurrentTask { get; private set; }
        public PoseDataset PoseDataset => poseDataset;

        private void Start()
        {
            if (loadOnStart)
            {
                LoadTask();
            }
        }

        public void SetPoseDataset(PoseDataset dataset)
        {
            poseDataset = dataset;
        }

        [ContextMenu("Load Robot Task Data")]
        public void LoadTask()
        {
            var poseRaw = ReadPoseSourceText(poseDataset);
            if (string.IsNullOrWhiteSpace(poseRaw))
            {
                Fail($"Pose JSON source is empty for dataset={poseDataset}.");
                return;
            }

            var trajectoryRaw = ReadSourceText(trajectoryCsv, trajectoryFallbackRelativePath);
            if (string.IsNullOrWhiteSpace(trajectoryRaw))
            {
                Fail("Trajectory CSV source is empty.");
                return;
            }

            var trajectorySamples = ParseTrajectoryCsv(trajectoryRaw);
            if (trajectorySamples.Count == 0)
            {
                Fail("No valid trajectory rows found in CSV.");
                return;
            }

            RobotTaskData loadedTask;
            switch (poseDataset)
            {
                case PoseDataset.TaskAPlane:
                case PoseDataset.TaskBCylinder:
                    if (!TryBuildTaskFromMotionEditorJson(poseRaw, trajectorySamples, out loadedTask, out var motionError))
                    {
                        Fail(motionError);
                        return;
                    }

                    break;
                case PoseDataset.TaskCCoordinateTransform:
            if (!TryBuildTaskFromCoordinateJson(poseRaw, trajectorySamples, out loadedTask, out var coordinateError))
                    {
                        Fail(coordinateError);
                        return;
                    }

                    break;
                default:
                    Fail($"Unsupported pose dataset type: {poseDataset}");
                    return;
            }

            CurrentTask = loadedTask;
            if (CurrentTask == null)
            {
                Fail("Failed to build task data from source payloads.");
                return;
            }

            if (logLoadedData)
            {
                Debug.Log(
                    $"[RobotTaskLoader] Loaded dataset={poseDataset}, task={CurrentTask.TaskId}, poses={CurrentTask.Poses.Count}, trajectorySamples={CurrentTask.TrajectorySamples.Count}");
            }

            OnTaskLoaded?.Invoke(CurrentTask);
        }

        private bool TryBuildTaskFromCoordinateJson(
            string poseRaw,
            List<JointTrajectorySampleData> trajectorySamples,
            out RobotTaskData taskData,
            out string error)
        {
            taskData = null;
            error = string.Empty;

            CoordinateTaskJson parsedPoseJson;
            try
            {
                parsedPoseJson = JsonUtility.FromJson<CoordinateTaskJson>(poseRaw);
            }
            catch (Exception exception)
            {
                error = $"Coordinate pose JSON parse error: {exception.Message}";
                return false;
            }

            if (parsedPoseJson == null || parsedPoseJson.posesInWorkpieceFrame == null || parsedPoseJson.posesInWorkpieceFrame.Length == 0)
            {
                error = "No poses found in coordinate payload (posesInWorkpieceFrame).";
                return false;
            }

            taskData = ConvertCoordinateTask(parsedPoseJson, trajectorySamples, taskCCoordinateWorldPoseOffset);
            if (taskData == null)
            {
                error = "Coordinate payload conversion failed.";
                return false;
            }

            return true;
        }

        private bool TryBuildTaskFromMotionEditorJson(
            string poseRaw,
            List<JointTrajectorySampleData> trajectorySamples,
            out RobotTaskData taskData,
            out string error)
        {
            taskData = null;
            error = string.Empty;

            MotionEditorTaskJson parsedPoseJson;
            try
            {
                parsedPoseJson = JsonUtility.FromJson<MotionEditorTaskJson>(poseRaw);
            }
            catch (Exception exception)
            {
                error = $"Motion editor pose JSON parse error: {exception.Message}";
                return false;
            }

            if (parsedPoseJson == null || parsedPoseJson.waypoints == null || parsedPoseJson.waypoints.Length == 0)
            {
                error = "No waypoints found in motion editor payload.";
                return false;
            }

            taskData = ConvertMotionEditorTask(parsedPoseJson, trajectorySamples);
            if (taskData == null)
            {
                error = "Motion editor payload conversion failed.";
                return false;
            }

            return true;
        }

        private RobotTaskData ConvertCoordinateTask(
            CoordinateTaskJson source,
            List<JointTrajectorySampleData> trajectorySamples,
            Vector3 worldPoseOffset)
        {
            var sourceAxis = new AxisConventionData(
                source != null && source.sourceFrame != null && source.sourceFrame.axisConvention != null ? source.sourceFrame.axisConvention.x : "forward",
                source != null && source.sourceFrame != null && source.sourceFrame.axisConvention != null ? source.sourceFrame.axisConvention.y : "left",
                source != null && source.sourceFrame != null && source.sourceFrame.axisConvention != null ? source.sourceFrame.axisConvention.z : "up");

            var unityAssumption = new AxisConventionData(
                source != null && source.unityFrameAssumption != null ? source.unityFrameAssumption.x : "right",
                source != null && source.unityFrameAssumption != null ? source.unityFrameAssumption.y : "up",
                source != null && source.unityFrameAssumption != null ? source.unityFrameAssumption.z : "forward");

            var workpieceTranslation = source != null && source.workpieceFrameInRobotBase != null
                ? ReadVector3(source.workpieceFrameInRobotBase.translation, Vector3.zero)
                : Vector3.zero;
            var workpieceRotationZyx = source != null && source.workpieceFrameInRobotBase != null
                ? ReadVector3(source.workpieceFrameInRobotBase.rotationEulerZYX, Vector3.zero)
                : Vector3.zero;

            var sourceToUnityBasis = CoordinateTransformUtility.BuildSourceToUnityBasis(sourceAxis, unityAssumption);
            var workpieceRotationSource = CoordinateTransformUtility.BuildRotationFromEulerZyx(workpieceRotationZyx);
            var workpieceTransformSource = CoordinateTransformUtility.BuildTransform(workpieceTranslation, workpieceRotationSource);

            var convertedPoses = new List<PoseData>();
            if (source != null && source.posesInWorkpieceFrame != null)
            {
                for (var i = 0; i < source.posesInWorkpieceFrame.Length; i++)
                {
                    var pose = source.posesInWorkpieceFrame[i];
                    if (pose == null)
                    {
                        continue;
                    }

                    var id = string.IsNullOrWhiteSpace(pose.id) ? $"P_{i:000}" : pose.id;
                    var localPosition = ReadVector3(pose.position, Vector3.zero);
                    var localRotationEuler = ReadVector3(pose.rotation, Vector3.zero);

                    var localRotationSource = CoordinateTransformUtility.BuildRotationFromEulerXyz(localRotationEuler);
                    var localPoseSource = CoordinateTransformUtility.BuildTransform(localPosition, localRotationSource);
                    var worldPoseSource = workpieceTransformSource * localPoseSource;
                    var worldPoseUnity = CoordinateTransformUtility.ConvertSourceMatrixToUnity(worldPoseSource, sourceToUnityBasis);

                    var worldPosition = CoordinateTransformUtility.ExtractPosition(worldPoseUnity) + worldPoseOffset;
                    convertedPoses.Add(new PoseData(
                        id,
                        localPosition,
                        localRotationEuler,
                        worldPosition,
                        CoordinateTransformUtility.ExtractRotation(worldPoseUnity)));
                }
            }

            return new RobotTaskData(
                source != null ? source.taskId : "TASK-C-UNKNOWN",
                source != null ? source.description : string.Empty,
                source != null && source.sourceFrame != null ? source.sourceFrame.name : "robot_base",
                sourceAxis,
                unityAssumption,
                workpieceTranslation,
                workpieceRotationZyx,
                source != null && source.units != null ? source.units.position : "meters",
                source != null && source.units != null ? source.units.rotation : "degrees",
                convertedPoses,
                trajectorySamples);
        }

        private static RobotTaskData ConvertMotionEditorTask(MotionEditorTaskJson source, List<JointTrajectorySampleData> trajectorySamples)
        {
            var sourceAxis = new AxisConventionData("right", "up", "forward");
            var unityAssumption = new AxisConventionData("right", "up", "forward");
            var convertedPoses = new List<PoseData>();
            if (source != null && source.waypoints != null)
            {
                for (var i = 0; i < source.waypoints.Length; i++)
                {
                    var waypoint = source.waypoints[i];
                    if (waypoint == null)
                    {
                        continue;
                    }

                    var id = string.IsNullOrWhiteSpace(waypoint.id) ? $"WP_{i:000}" : waypoint.id;
                    var localPosition = ReadVector3(waypoint.position, Vector3.zero);
                    var localRotationEuler = ReadVector3(waypoint.rotation, Vector3.zero);
                    convertedPoses.Add(new PoseData(
                        id,
                        localPosition,
                        localRotationEuler,
                        localPosition,
                        Quaternion.Euler(localRotationEuler)));
                }
            }

            var taskIdFallback = "TASK-MOTION-UNKNOWN";
            if (source != null)
            {
                taskIdFallback = string.Equals(source.surfaceType, "Cylinder", StringComparison.OrdinalIgnoreCase)
                    ? "TASK-B-CYLINDER-OUTER-SWEEP"
                    : "TASK-A-PLANE-SWEEP";
            }

            var description = source != null ? source.description : string.Empty;
            if (!string.IsNullOrWhiteSpace(source != null ? source.surfaceType : string.Empty))
            {
                description = $"{description} (surface={source.surfaceType})";
            }

            return new RobotTaskData(
                source != null && !string.IsNullOrWhiteSpace(source.taskId) ? source.taskId : taskIdFallback,
                description,
                source != null && !string.IsNullOrWhiteSpace(source.frame) ? source.frame : "workpiece",
                sourceAxis,
                unityAssumption,
                Vector3.zero,
                Vector3.zero,
                source != null && source.units != null ? source.units.position : "meters",
                source != null && source.units != null ? source.units.rotation : "degrees",
                convertedPoses,
                trajectorySamples);
        }

        private string ReadPoseSourceText(PoseDataset dataset)
        {
            switch (dataset)
            {
                case PoseDataset.TaskAPlane:
                    return ReadSourceText(taskAPlanePoseJson, taskAPlaneFallbackRelativePath);
                case PoseDataset.TaskBCylinder:
                    return ReadSourceText(taskBCylinderPoseJson, taskBCylinderFallbackRelativePath);
                case PoseDataset.TaskCCoordinateTransform:
                    return ReadSourceText(taskCCoordinatePoseJson, taskCCoordinateFallbackRelativePath);
                default:
                    return string.Empty;
            }
        }

        private static List<JointTrajectorySampleData> ParseTrajectoryCsv(string csv)
        {
            var rows = new List<JointTrajectorySampleData>();
            if (string.IsNullOrWhiteSpace(csv))
            {
                return rows;
            }

            var lines = csv.Split('\n');
            if (lines.Length < 2)
            {
                return rows;
            }

            var header = SplitCsvLine(lines[0]);
            var timeIndex = FindHeaderIndex(header, "time_sec", "time");
            var j1Index = FindHeaderIndex(header, "j1_deg", "j1");
            var j2Index = FindHeaderIndex(header, "j2_deg", "j2");
            var j3Index = FindHeaderIndex(header, "j3_deg", "j3");
            var j4Index = FindHeaderIndex(header, "j4_deg", "j4");
            var j5Index = FindHeaderIndex(header, "j5_deg", "j5");
            var j6Index = FindHeaderIndex(header, "j6_deg", "j6");
            var segmentIndex = FindHeaderIndex(header, "segment");

            if (timeIndex < 0 || j1Index < 0 || j2Index < 0 || j3Index < 0 || j4Index < 0 || j5Index < 0 || j6Index < 0)
            {
                return rows;
            }

            for (var i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i]))
                {
                    continue;
                }

                var columns = SplitCsvLine(lines[i]);
                if (!TryParseFloat(GetColumn(columns, timeIndex), out var timeSec))
                {
                    continue;
                }

                var joints = new float[6];
                if (!TryParseFloat(GetColumn(columns, j1Index), out joints[0])
                    || !TryParseFloat(GetColumn(columns, j2Index), out joints[1])
                    || !TryParseFloat(GetColumn(columns, j3Index), out joints[2])
                    || !TryParseFloat(GetColumn(columns, j4Index), out joints[3])
                    || !TryParseFloat(GetColumn(columns, j5Index), out joints[4])
                    || !TryParseFloat(GetColumn(columns, j6Index), out joints[5]))
                {
                    continue;
                }

                var segment = segmentIndex >= 0 ? GetColumn(columns, segmentIndex) : "-";
                rows.Add(new JointTrajectorySampleData(timeSec, joints, segment));
            }

            rows.Sort((a, b) => a.TimeSec.CompareTo(b.TimeSec));
            return rows;
        }

        private static string[] SplitCsvLine(string line)
        {
            return string.IsNullOrWhiteSpace(line)
                ? Array.Empty<string>()
                : line.Replace("\r", string.Empty).Split(',');
        }

        private static int FindHeaderIndex(string[] headerColumns, params string[] candidates)
        {
            if (headerColumns == null || headerColumns.Length == 0)
            {
                return -1;
            }

            for (var i = 0; i < headerColumns.Length; i++)
            {
                var value = headerColumns[i].Trim();
                for (var j = 0; j < candidates.Length; j++)
                {
                    if (string.Equals(value, candidates[j], StringComparison.OrdinalIgnoreCase))
                    {
                        return i;
                    }
                }
            }

            return -1;
        }

        private static bool TryParseFloat(string value, out float result)
        {
            return float.TryParse(
                value,
                NumberStyles.Float | NumberStyles.AllowThousands,
                CultureInfo.InvariantCulture,
                out result);
        }

        private static string GetColumn(string[] columns, int index)
        {
            if (columns == null || index < 0 || index >= columns.Length)
            {
                return string.Empty;
            }

            return columns[index].Trim();
        }

        private static Vector3 ReadVector3(float[] values, Vector3 fallback)
        {
            return new Vector3(
                values != null && values.Length > 0 ? values[0] : fallback.x,
                values != null && values.Length > 1 ? values[1] : fallback.y,
                values != null && values.Length > 2 ? values[2] : fallback.z);
        }

        private static string ReadSourceText(TextAsset textAsset, string fallbackRelativePath)
        {
            if (textAsset != null && !string.IsNullOrWhiteSpace(textAsset.text))
            {
                return textAsset.text;
            }

            if (string.IsNullOrWhiteSpace(fallbackRelativePath))
            {
                return string.Empty;
            }

            var fullPath = Path.IsPathRooted(fallbackRelativePath)
                ? fallbackRelativePath
                : Path.Combine(Application.dataPath, fallbackRelativePath);

            if (!File.Exists(fullPath))
            {
                return string.Empty;
            }

            try
            {
                return File.ReadAllText(fullPath);
            }
            catch
            {
                return string.Empty;
            }
        }

        private void Fail(string reason)
        {
            var message = $"[RobotTaskLoader] {reason}";
            Debug.LogError(message);
            OnTaskLoadFailed?.Invoke(message);
        }
    }
}
