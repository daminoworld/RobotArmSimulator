using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotArmSimulator
{
    [Serializable]
    internal sealed class CoordinateUnitsJson
    {
        public string position;
        public string rotation;
    }

    [Serializable]
    internal sealed class CoordinateAxisConventionJson
    {
        public string x;
        public string y;
        public string z;
    }

    [Serializable]
    internal sealed class CoordinateSourceFrameJson
    {
        public string name;
        public CoordinateAxisConventionJson axisConvention;
    }

    [Serializable]
    internal sealed class CoordinateWorkpieceFrameJson
    {
        public float[] translation;
        public float[] rotationEulerZYX;
    }

    [Serializable]
    internal sealed class CoordinateUnityFrameAssumptionJson
    {
        public string x;
        public string y;
        public string z;
    }

    [Serializable]
    internal sealed class CoordinatePoseJson
    {
        public string id;
        public float[] position;
        public float[] rotation;
    }

    [Serializable]
    internal sealed class CoordinateTaskJson
    {
        public string taskId;
        public string description;
        public CoordinateUnitsJson units;
        public CoordinateSourceFrameJson sourceFrame;
        public CoordinateWorkpieceFrameJson workpieceFrameInRobotBase;
        public CoordinateUnityFrameAssumptionJson unityFrameAssumption;
        public CoordinatePoseJson[] posesInWorkpieceFrame;
    }

    [Serializable]
    internal sealed class MotionEditorWaypointJson
    {
        public string id;
        public float[] position;
        public float[] rotation;
        public float speed;
        public float dwellSec;
        public string label;
        public string toolState;
        public float[] surfaceNormal;
    }

    [Serializable]
    internal sealed class MotionEditorTaskJson
    {
        public string taskId;
        public string description;
        public string surfaceType;
        public string frame;
        public CoordinateUnitsJson units;
        public MotionEditorWaypointJson[] waypoints;
    }

    public sealed class AxisConventionData
    {
        public AxisConventionData(string x, string y, string z)
        {
            X = string.IsNullOrWhiteSpace(x) ? "right" : x;
            Y = string.IsNullOrWhiteSpace(y) ? "up" : y;
            Z = string.IsNullOrWhiteSpace(z) ? "forward" : z;
        }

        public string X { get; }
        public string Y { get; }
        public string Z { get; }
    }

    public sealed class PoseData
    {
        public PoseData(
            string id,
            Vector3 positionInWorkpiece,
            Vector3 rotationEulerInWorkpiece,
            Vector3 worldPosition,
            Quaternion worldRotation)
        {
            Id = string.IsNullOrWhiteSpace(id) ? "P_UNKNOWN" : id;
            PositionInWorkpiece = positionInWorkpiece;
            RotationEulerInWorkpiece = rotationEulerInWorkpiece;
            WorldPosition = worldPosition;
            WorldRotation = worldRotation;
        }

        public string Id { get; }
        public Vector3 PositionInWorkpiece { get; }
        public Vector3 RotationEulerInWorkpiece { get; }
        public Vector3 WorldPosition { get; }
        public Quaternion WorldRotation { get; }
    }

    public sealed class JointTrajectorySampleData
    {
        public JointTrajectorySampleData(float timeSec, float[] jointAnglesDeg, string segment)
        {
            TimeSec = Mathf.Max(0f, timeSec);
            JointAnglesDeg = new float[6];
            for (var i = 0; i < JointAnglesDeg.Length; i++)
            {
                JointAnglesDeg[i] = jointAnglesDeg != null && i < jointAnglesDeg.Length ? jointAnglesDeg[i] : 0f;
            }

            Segment = string.IsNullOrWhiteSpace(segment) ? "-" : segment.Trim();
        }

        public float TimeSec { get; }
        public float[] JointAnglesDeg { get; }
        public string Segment { get; }
    }

    public sealed class RobotTaskData
    {
        private readonly List<PoseData> _poses;
        private readonly List<JointTrajectorySampleData> _trajectorySamples;

        public RobotTaskData(
            string taskId,
            string description,
            string sourceFrameName,
            AxisConventionData sourceAxisConvention,
            AxisConventionData unityFrameAssumption,
            Vector3 workpieceTranslationInRobotBase,
            Vector3 workpieceRotationEulerZyx,
            string positionUnit,
            string rotationUnit,
            List<PoseData> poses,
            List<JointTrajectorySampleData> trajectorySamples)
        {
            TaskId = string.IsNullOrWhiteSpace(taskId) ? "TASK-C-UNKNOWN" : taskId;
            Description = string.IsNullOrWhiteSpace(description) ? "-" : description;
            SourceFrameName = string.IsNullOrWhiteSpace(sourceFrameName) ? "robot_base" : sourceFrameName;
            SourceAxisConvention = sourceAxisConvention ?? new AxisConventionData("forward", "left", "up");
            UnityFrameAssumption = unityFrameAssumption ?? new AxisConventionData("right", "up", "forward");
            WorkpieceTranslationInRobotBase = workpieceTranslationInRobotBase;
            WorkpieceRotationEulerZyx = workpieceRotationEulerZyx;
            PositionUnit = string.IsNullOrWhiteSpace(positionUnit) ? "meters" : positionUnit;
            RotationUnit = string.IsNullOrWhiteSpace(rotationUnit) ? "degrees" : rotationUnit;
            _poses = poses ?? new List<PoseData>();
            _trajectorySamples = trajectorySamples ?? new List<JointTrajectorySampleData>();
        }

        public string TaskId { get; }
        public string Description { get; }
        public string SourceFrameName { get; }
        public AxisConventionData SourceAxisConvention { get; }
        public AxisConventionData UnityFrameAssumption { get; }
        public Vector3 WorkpieceTranslationInRobotBase { get; }
        public Vector3 WorkpieceRotationEulerZyx { get; }
        public string PositionUnit { get; }
        public string RotationUnit { get; }
        public IReadOnlyList<PoseData> Poses => _poses;
        public IReadOnlyList<JointTrajectorySampleData> TrajectorySamples => _trajectorySamples;
    }
}
