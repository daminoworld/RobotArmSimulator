using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace RobotArmSimulator
{
    public sealed class SimulatorUIController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private RobotTaskLoader taskLoader;
        [SerializeField] private PoseTrajectoryRenderer poseRenderer;
        [SerializeField] private JointTrajectoryPlaybackController playbackController;
        [SerializeField] private SimpleJointRobotVisualizer robotVisualizer;
        [SerializeField] private Simple6AxisIkSolver ccdSolver;
        [SerializeField] private FabrikIkSolver fabrikSolver;
        [SerializeField] private OrbitCameraController cameraController;

        [Header("Robot Placement")]
        [SerializeField] private Transform taskABaseReference;
        [SerializeField] private Transform taskBBaseReference;
        [SerializeField] private Transform taskCBaseReference;
        [SerializeField] private Vector3 taskAPlaneRobotBaseOffset = new Vector3(0f, -0.18f, -0.34f);
        [SerializeField] private Vector3 taskBCylinderRobotBaseOffset = new Vector3(0.06f, -0.2f, -0.36f);
        [SerializeField] private Vector3 taskCCoordinateRobotBaseOffset = new Vector3(-0.12f, -0.22f, -0.42f);

        [Header("UI Names")]
        [SerializeField] private string taskIdValueLabelName = "taskIdValue";
        [SerializeField] private string poseCountValueLabelName = "poseCountValue";
        [SerializeField] private string trajectoryCountValueLabelName = "trajectoryCountValue";
        [SerializeField] private string sourceFrameValueLabelName = "sourceFrameValue";
        [SerializeField] private string statusLabelName = "statusLabel";
        [SerializeField] private string poseDatasetDropdownName = "poseDatasetDropdown";
        [SerializeField] private string playbackStateLabelName = "playbackStateLabel";
        [SerializeField] private string timestampLabelName = "timestampLabel";
        [SerializeField] private string segmentLabelName = "segmentLabel";
        [SerializeField] private string jointValuesLabelName = "jointValuesLabel";
        [SerializeField] private string waypointListName = "waypointList";
        [SerializeField] private string playButtonName = "playButton";
        [SerializeField] private string pauseButtonName = "pauseButton";
        [SerializeField] private string stopButtonName = "stopButton";
        [SerializeField] private string reloadButtonName = "reloadButton";
        [SerializeField] private string ikSolverDropdownName = "ikSolverDropdown";
        [SerializeField] private string selectedIdLabelName = "selectedIdLabel";
        [SerializeField] private string selectedLocalPositionLabelName = "selectedLocalPositionLabel";
        [SerializeField] private string selectedLocalRotationLabelName = "selectedLocalRotationLabel";
        [SerializeField] private string selectedWorldPositionLabelName = "selectedWorldPositionLabel";
        [SerializeField] private string selectedWorldRotationLabelName = "selectedWorldRotationLabel";
        [SerializeField] private string jointAnglesLabelName = "jointAnglesLabel";

        private readonly List<string> _waypointItems = new List<string>();

        private IIkSolver _activeIkSolver;
        private DropdownField _ikSolverDropdown;

        private static readonly List<string> IkSolverChoices = new List<string> { "CCD", "FABRIK" };

        private Label _taskIdValueLabel;
        private Label _poseCountValueLabel;
        private Label _trajectoryCountValueLabel;
        private Label _sourceFrameValueLabel;
        private Label _statusLabel;
        private DropdownField _poseDatasetDropdown;
        private Label _playbackStateLabel;
        private Label _timestampLabel;
        private Label _segmentLabel;
        private Label _jointValuesLabel;
        private ListView _waypointList;
        private Button _playButton;
        private Button _pauseButton;
        private Button _stopButton;
        private Button _reloadButton;
        private Label _selectedIdLabel;
        private Label _selectedLocalPositionLabel;
        private Label _selectedLocalRotationLabel;
        private Label _selectedWorldPositionLabel;
        private Label _selectedWorldRotationLabel;
        private Label _jointAnglesLabel;
        private float _jointAnglesRefreshTimer;
        private const float JointAnglesRefreshInterval = 0.05f;

        private RobotTaskData _taskData;
        private int _selectedIndex = -1;
        private bool _bound;
        private bool _suppressSelectionSync;

        private static readonly List<string> PoseDatasetChoices = new List<string>
        {
            "Task A - Plane",
            "Task B - Cylinder",
            "Task C - Coordinate"
        };

        private void Awake()
        {
            if (uiDocument == null)
            {
                uiDocument = GetComponent<UIDocument>();
            }

            if (taskLoader == null)
            {
                taskLoader = GetComponentInChildren<RobotTaskLoader>();
            }

            if (poseRenderer == null)
            {
                poseRenderer = GetComponentInChildren<PoseTrajectoryRenderer>();
            }

            if (playbackController == null)
            {
                playbackController = GetComponentInChildren<JointTrajectoryPlaybackController>();
            }

            if (robotVisualizer == null)
            {
                robotVisualizer = GetComponentInChildren<SimpleJointRobotVisualizer>();
            }

            if (ccdSolver == null)
            {
                ccdSolver = GetComponentInChildren<Simple6AxisIkSolver>();
            }

            if (fabrikSolver == null)
            {
                fabrikSolver = GetComponentInChildren<FabrikIkSolver>();
            }
        }

        private void OnEnable()
        {
            CacheUiElements();
            BindUiEvents();
            SubscribeExternalEvents();
            RenderEmptyState();

            if (cameraController != null)
            {
                cameraController.IsPointerBlocked = () =>
                    IsPointerOverInteractivePanel(PointerInput.GetPointerScreenPosition());
            }
        }

        private void Start()
        {
            if (taskLoader == null)
            {
                SetStatus("Task loader is missing.");
                return;
            }

            if (taskLoader.CurrentTask != null)
            {
                ApplyTask(taskLoader.CurrentTask);
            }
            else
            {
                taskLoader.LoadTask();
            }
        }

        private void Update()
        {
            _jointAnglesRefreshTimer -= Time.deltaTime;
            if (_jointAnglesRefreshTimer > 0f) return;
            _jointAnglesRefreshTimer = JointAnglesRefreshInterval;
            RefreshJointAnglesLabel();
        }

        private void OnDisable()
        {
            UnbindUiEvents();
            UnsubscribeExternalEvents();
        }

        private void CacheUiElements()
        {
            if (uiDocument == null || uiDocument.rootVisualElement == null)
            {
                return;
            }

            var root = uiDocument.rootVisualElement;
            _taskIdValueLabel = root.Q<Label>(taskIdValueLabelName);
            _poseCountValueLabel = root.Q<Label>(poseCountValueLabelName);
            _trajectoryCountValueLabel = root.Q<Label>(trajectoryCountValueLabelName);
            _sourceFrameValueLabel = root.Q<Label>(sourceFrameValueLabelName);
            _statusLabel = root.Q<Label>(statusLabelName);
            _poseDatasetDropdown = root.Q<DropdownField>(poseDatasetDropdownName);
            _playbackStateLabel = root.Q<Label>(playbackStateLabelName);
            _timestampLabel = root.Q<Label>(timestampLabelName);
            _segmentLabel = root.Q<Label>(segmentLabelName);
            _jointValuesLabel = root.Q<Label>(jointValuesLabelName);
            _waypointList = root.Q<ListView>(waypointListName);
            _playButton = root.Q<Button>(playButtonName);
            _pauseButton = root.Q<Button>(pauseButtonName);
            _stopButton = root.Q<Button>(stopButtonName);
            _reloadButton = root.Q<Button>(reloadButtonName);
            _selectedIdLabel = root.Q<Label>(selectedIdLabelName);
            _selectedLocalPositionLabel = root.Q<Label>(selectedLocalPositionLabelName);
            _selectedLocalRotationLabel = root.Q<Label>(selectedLocalRotationLabelName);
            _selectedWorldPositionLabel = root.Q<Label>(selectedWorldPositionLabelName);
            _selectedWorldRotationLabel = root.Q<Label>(selectedWorldRotationLabelName);
            _jointAnglesLabel = root.Q<Label>(jointAnglesLabelName);

            if (_poseDatasetDropdown != null)
            {
                _poseDatasetDropdown.choices = PoseDatasetChoices;
                RefreshPoseDatasetDropdown();
            }

            _ikSolverDropdown = root.Q<DropdownField>(ikSolverDropdownName);
            if (_ikSolverDropdown != null)
            {
                _ikSolverDropdown.choices = IkSolverChoices;
                _ikSolverDropdown.SetValueWithoutNotify(IkSolverChoices[0]);
            }

            SetActiveSolver(ccdSolver);

            if (_waypointList != null)
            {
                _waypointList.selectionType = SelectionType.Single;
                _waypointList.itemHeight = 30;
                _waypointList.makeItem = () =>
                {
                    var label = new Label("Waypoint");
                    label.AddToClassList("waypoint-item");
                    return label;
                };
                _waypointList.bindItem = (element, index) =>
                {
                    if (element is Label label && index >= 0 && index < _waypointItems.Count)
                    {
                        label.text = _waypointItems[index];
                        label.EnableInClassList("selected", index == _selectedIndex);
                    }
                };
                _waypointList.itemsSource = _waypointItems;
            }
        }

        private void BindUiEvents()
        {
            if (_bound)
            {
                return;
            }

            if (_waypointList != null)
            {
#if UNITY_2022_2_OR_NEWER
                _waypointList.selectionChanged += OnWaypointListSelectionChanged;
#elif UNITY_2020_1_OR_NEWER
                _waypointList.onSelectionChange += OnWaypointListSelectionChanged;
#else
                _waypointList.onSelectionChanged += OnWaypointListSelectionChanged;
#endif
            }

            if (_playButton != null)
            {
                _playButton.clicked += OnPlayClicked;
            }

            if (_pauseButton != null)
            {
                _pauseButton.clicked += OnPauseClicked;
            }

            if (_stopButton != null)
            {
                _stopButton.clicked += OnStopClicked;
            }

            if (_reloadButton != null)
            {
                _reloadButton.clicked += OnReloadClicked;
            }

            if (_poseDatasetDropdown != null)
            {
                _poseDatasetDropdown.RegisterValueChangedCallback(OnPoseDatasetChanged);
            }

            if (_ikSolverDropdown != null)
            {
                _ikSolverDropdown.RegisterValueChangedCallback(OnIkSolverChanged);
            }

            _bound = true;
        }

        private void UnbindUiEvents()
        {
            if (!_bound)
            {
                return;
            }

            if (_waypointList != null)
            {
#if UNITY_2022_2_OR_NEWER
                _waypointList.selectionChanged -= OnWaypointListSelectionChanged;
#elif UNITY_2020_1_OR_NEWER
                _waypointList.onSelectionChange -= OnWaypointListSelectionChanged;
#else
                _waypointList.onSelectionChanged -= OnWaypointListSelectionChanged;
#endif
            }

            if (_playButton != null)
            {
                _playButton.clicked -= OnPlayClicked;
            }

            if (_pauseButton != null)
            {
                _pauseButton.clicked -= OnPauseClicked;
            }

            if (_stopButton != null)
            {
                _stopButton.clicked -= OnStopClicked;
            }

            if (_reloadButton != null)
            {
                _reloadButton.clicked -= OnReloadClicked;
            }

            if (_poseDatasetDropdown != null)
            {
                _poseDatasetDropdown.UnregisterValueChangedCallback(OnPoseDatasetChanged);
            }

            if (_ikSolverDropdown != null)
            {
                _ikSolverDropdown.UnregisterValueChangedCallback(OnIkSolverChanged);
            }

            _bound = false;
        }

        private bool IsPointerOverInteractivePanel(Vector2 screenPosition)
        {
            var root = uiDocument?.rootVisualElement;
            var panel = root?.panel;
            if (panel == null)
            {
                return false;
            }

            var panelPosition = RuntimePanelUtils.ScreenToPanel(panel, screenPosition);
            var picked = panel.Pick(panelPosition);
            return picked != null && picked.pickingMode != PickingMode.Ignore;
        }

        private void SubscribeExternalEvents()
        {
            UnsubscribeExternalEvents();

            if (taskLoader != null)
            {
                taskLoader.OnTaskLoaded += OnTaskLoaded;
                taskLoader.OnTaskLoadFailed += OnTaskLoadFailed;
            }

            if (playbackController != null)
            {
                playbackController.OnFrameUpdated += OnPlaybackFrameUpdated;
                playbackController.OnPlaybackStateChanged += OnPlaybackStateChanged;
                playbackController.OnPlaybackCompleted += OnPlaybackCompleted;
            }
        }

        private void UnsubscribeExternalEvents()
        {
            if (taskLoader != null)
            {
                taskLoader.OnTaskLoaded -= OnTaskLoaded;
                taskLoader.OnTaskLoadFailed -= OnTaskLoadFailed;
            }

            if (playbackController != null)
            {
                playbackController.OnFrameUpdated -= OnPlaybackFrameUpdated;
                playbackController.OnPlaybackStateChanged -= OnPlaybackStateChanged;
                playbackController.OnPlaybackCompleted -= OnPlaybackCompleted;
            }
        }

        private void OnTaskLoaded(RobotTaskData taskData)
        {
            ApplyTask(taskData);
        }

        private void OnTaskLoadFailed(string message)
        {
            SetStatus("Load failed");
            Debug.LogError(message);
        }

        private void ApplyTask(RobotTaskData taskData)
        {
            _taskData = taskData;
            _selectedIndex = -1;

            if (poseRenderer != null)
            {
                poseRenderer.SetMarkerRotationOffset(GetMarkerRotationOffsetForCurrentDataset());
                poseRenderer.Render(_taskData != null ? _taskData.Poses : null);
            }

            if (playbackController != null)
            {
                playbackController.SetData(
                    _taskData != null ? _taskData.TrajectorySamples : null,
                    _taskData != null ? _taskData.Poses : null);
            }

            if (robotVisualizer != null)
            {
                PlaceRobotBaseForCurrentTask();
                robotVisualizer.ResetToHomePose();
            }

            RefreshSummaryLabels();
            RefreshWaypointList();
            SetSelectedIndex(_taskData != null && _taskData.Poses.Count > 0 ? 0 : -1, true);
            _activeIkSolver?.SolveImmediately();
            RefreshPoseDatasetDropdown();
            SetStatus(_taskData != null ? $"Loaded {_taskData.TaskId}" : "No data loaded");
            UpdatePlaybackLabels(playbackController != null ? playbackController.CurrentFrame : null);
            UpdatePlaybackButtons();
        }

        private void OnWaypointListSelectionChanged(IEnumerable<object> selectedItems)
        {
            _ = selectedItems;
            if (_suppressSelectionSync || _waypointList == null)
            {
                return;
            }

            SetSelectedIndex(_waypointList.selectedIndex, false);
            _activeIkSolver?.SolveImmediately();
        }

        private void SetSelectedIndex(int index, bool forceUiRefresh, bool syncToolMarker = true)
        {
            if (_taskData == null || _taskData.Poses.Count == 0 || index < 0 || index >= _taskData.Poses.Count)
            {
                _selectedIndex = -1;
                RefreshWaypointListSelection(forceUiRefresh);
                RefreshSelectedPoseInfo(null);
                if (poseRenderer != null)
                {
                    poseRenderer.Highlight(-1);
                }

                return;
            }

            _selectedIndex = index;
            RefreshWaypointListSelection(forceUiRefresh);
            var selectedPose = _taskData.Poses[index];
            RefreshSelectedPoseInfo(selectedPose);
            poseRenderer?.Highlight(index);

            if (syncToolMarker && robotVisualizer != null && selectedPose != null)
            {
                robotVisualizer.SetToolTargetPose(selectedPose.WorldPosition, selectedPose.WorldRotation);
            }
        }

        private void RefreshSummaryLabels()
        {
            if (_taskData == null)
            {
                SetLabel(_taskIdValueLabel, "Task: -");
                SetLabel(_poseCountValueLabel, "Pose Count: 0");
                SetLabel(_trajectoryCountValueLabel, "Trajectory Samples: 0");
                SetLabel(_sourceFrameValueLabel, "Source Frame: -");
                return;
            }

            SetLabel(_taskIdValueLabel, $"Task: {_taskData.TaskId}");
            SetLabel(_poseCountValueLabel, $"Pose Count: {_taskData.Poses.Count}");
            SetLabel(_trajectoryCountValueLabel, $"Trajectory Samples: {_taskData.TrajectorySamples.Count}");
            SetLabel(_sourceFrameValueLabel, $"Source Frame: {_taskData.SourceFrameName}");
        }

        private void RefreshWaypointList()
        {
            _waypointItems.Clear();
            if (_taskData != null)
            {
                for (var i = 0; i < _taskData.Poses.Count; i++)
                {
                    var pose = _taskData.Poses[i];
                    if (pose == null)
                    {
                        continue;
                    }

                    _waypointItems.Add($"{i:00}  {pose.Id}");
                }
            }

            if (_waypointList == null)
            {
                return;
            }

            var previousSuppressState = _suppressSelectionSync;
            _suppressSelectionSync = true;
            try
            {
                _waypointList.itemsSource = null;
                _waypointList.itemsSource = new List<string>(_waypointItems);
                _waypointList.RefreshItems();
            }
            finally
            {
                _suppressSelectionSync = previousSuppressState;
            }
        }

        private void RefreshWaypointListSelection(bool forceRefresh)
        {
            if (_waypointList == null)
            {
                return;
            }

            var previousSuppressState = _suppressSelectionSync;
            _suppressSelectionSync = true;

            if (_selectedIndex < 0 || _selectedIndex >= _waypointItems.Count)
            {
                _waypointList.ClearSelection();
            }
            else
            {
#if UNITY_2022_2_OR_NEWER
                _waypointList.selectedIndex = _selectedIndex;
                _waypointList.ScrollToItem(_selectedIndex);
#else
                _waypointList.SetSelection(new[] { _selectedIndex });
#endif
            }

            if (forceRefresh)
            {
                _waypointList.RefreshItems();
            }

            _suppressSelectionSync = previousSuppressState;
        }

        private void RefreshSelectedPoseInfo(PoseData pose)
        {
            SetLabel(_selectedIdLabel, $"id: {(pose != null ? pose.Id : "-")}");
            SetLabel(
                _selectedLocalPositionLabel,
                $"workpiecePosition: {(pose != null ? FormatVector(pose.PositionInWorkpiece) : "-")}");
            SetLabel(
                _selectedLocalRotationLabel,
                $"workpieceRotation: {(pose != null ? FormatVector(pose.RotationEulerInWorkpiece) : "-")}");
            SetLabel(
                _selectedWorldPositionLabel,
                $"worldPosition: {(pose != null ? FormatVector(pose.WorldPosition) : "-")}");
            SetLabel(
                _selectedWorldRotationLabel,
                $"worldRotation: {(pose != null ? FormatVector(pose.WorldRotation.eulerAngles) : "-")}");
        }

        private void OnPlaybackFrameUpdated(JointTrajectoryFrameData frame)
        {
            if (frame == null)
            {
                UpdatePlaybackLabels(null);
                return;
            }

            if (robotVisualizer != null)
            {
                if (frame.HasTargetPose)
                {
                    robotVisualizer.SetToolTargetPose(frame.TargetPosition, frame.TargetRotation);
                }

                if (playbackController != null
                    && playbackController.PlaybackMode == JointTrajectoryPlaybackMode.JointCsv
                    && frame.HasJointValues)
                {
                    robotVisualizer.ApplyJointAngles(frame.JointAnglesDeg);
                }
            }

            if (frame.WaypointIndex >= 0 && frame.WaypointIndex != _selectedIndex)
            {
                SetSelectedIndex(frame.WaypointIndex, false, false);
            }

            UpdatePlaybackLabels(frame);
        }

        private void OnPlaybackStateChanged(JointTrajectoryPlaybackState state)
        {
            SetLabel(_playbackStateLabel, $"Playback: {state}");
            UpdatePlaybackButtons();
        }

        private void OnPlaybackCompleted()
        {
            UpdatePlaybackButtons();
        }

        private void UpdatePlaybackLabels(JointTrajectoryFrameData frame)
        {
            if (frame == null)
            {
                SetLabel(_timestampLabel, "time: -");
                SetLabel(_segmentLabel, "segment: -");
                SetLabel(_jointValuesLabel, "joints: -");
                return;
            }

            SetLabel(_timestampLabel, $"time: {frame.TimeSec:0.000} s");
            SetLabel(_segmentLabel, $"segment: {frame.Segment}");
            var jointsForDisplay = robotVisualizer != null ? robotVisualizer.GetCurrentJointAnglesCopy() : frame.JointAnglesDeg;
            SetLabel(_jointValuesLabel, FormatJointValues(jointsForDisplay));
        }

        private void OnPlayClicked()
        {
            playbackController?.Play();
            UpdatePlaybackButtons();
        }

        private void OnPauseClicked()
        {
            playbackController?.Pause();
            UpdatePlaybackButtons();
        }

        private void OnStopClicked()
        {
            playbackController?.Stop();
            _activeIkSolver?.SolveImmediately();
            UpdatePlaybackButtons();
        }

        private void OnReloadClicked()
        {
            taskLoader?.LoadTask();
        }

        private void OnIkSolverChanged(ChangeEvent<string> evt)
        {
            var incoming = string.Equals(evt.newValue, IkSolverChoices[1], StringComparison.Ordinal)
                ? (MonoBehaviour)fabrikSolver
                : (MonoBehaviour)ccdSolver;
            SetActiveSolver(incoming);
            _activeIkSolver?.SolveImmediately();
        }

        private void SetActiveSolver(MonoBehaviour incoming)
        {
            var outgoing = ReferenceEquals(incoming, ccdSolver)
                ? (MonoBehaviour)fabrikSolver
                : (MonoBehaviour)ccdSolver;

            if (outgoing != null) outgoing.enabled = false;
            if (incoming != null) incoming.enabled = true;

            _activeIkSolver = incoming as IIkSolver;
        }

        private void OnPoseDatasetChanged(ChangeEvent<string> evt)
        {
            _ = evt;
            if (taskLoader == null || _poseDatasetDropdown == null)
            {
                return;
            }

            var nextDataset = DropdownValueToDataset(_poseDatasetDropdown.value);
            if (nextDataset == taskLoader.PoseDataset)
            {
                return;
            }

            playbackController?.Stop();
            taskLoader.SetPoseDataset(nextDataset);
            SetStatus($"Loading {DatasetToDropdownValue(nextDataset)}");
            taskLoader.LoadTask();
        }

        private void UpdatePlaybackButtons()
        {
            var hasPlaybackData = playbackController != null
                ? playbackController.HasPlayableData
                : (_taskData != null && _taskData.Poses.Count > 0);
            var state = playbackController != null ? playbackController.State : JointTrajectoryPlaybackState.Stopped;

            if (_playButton != null)
            {
                _playButton.SetEnabled(hasPlaybackData && state != JointTrajectoryPlaybackState.Playing);
            }

            if (_pauseButton != null)
            {
                _pauseButton.SetEnabled(hasPlaybackData && state == JointTrajectoryPlaybackState.Playing);
            }

            if (_stopButton != null)
            {
                _stopButton.SetEnabled(hasPlaybackData && state != JointTrajectoryPlaybackState.Stopped);
            }

            if (_reloadButton != null)
            {
                _reloadButton.SetEnabled(taskLoader != null);
            }

            if (_waypointList != null)
            {
                _waypointList.SetEnabled(_waypointItems.Count > 0);
            }
        }

        private void RenderEmptyState()
        {
            RefreshSummaryLabels();
            RefreshSelectedPoseInfo(null);
            RefreshPoseDatasetDropdown();
            SetLabel(_playbackStateLabel, "Playback: Stopped");
            UpdatePlaybackLabels(null);
            SetStatus("Waiting for data");
            UpdatePlaybackButtons();
        }

        private void RefreshJointAnglesLabel()
        {
            if (robotVisualizer == null) return;
            SetLabel(_jointAnglesLabel, FormatJointValues(robotVisualizer.GetCurrentJointAnglesCopy()));
        }

        private void SetStatus(string status)
        {
            SetLabel(_statusLabel, $"Status: {status}");
        }

        private static string FormatJointValues(float[] joints)
        {
            if (joints == null || joints.Length == 0)
            {
                return "joints: -";
            }

            return
                $"j1: {GetJoint(joints, 0):0.##}   j2: {GetJoint(joints, 1):0.##}   j3: {GetJoint(joints, 2):0.##}\n" +
                $"j4: {GetJoint(joints, 3):0.##}   j5: {GetJoint(joints, 4):0.##}   j6: {GetJoint(joints, 5):0.##}";
        }

        private static float GetJoint(float[] joints, int index)
        {
            return joints != null && index >= 0 && index < joints.Length ? joints[index] : 0f;
        }

        private static string FormatVector(Vector3 value)
        {
            return $"({value.x:0.###}, {value.y:0.###}, {value.z:0.###})";
        }

        private static void SetLabel(Label label, string text)
        {
            if (label != null)
            {
                label.text = text;
            }
        }

        private void RefreshPoseDatasetDropdown()
        {
            if (_poseDatasetDropdown == null || taskLoader == null)
            {
                return;
            }

            _poseDatasetDropdown.choices = PoseDatasetChoices;
            _poseDatasetDropdown.SetValueWithoutNotify(DatasetToDropdownValue(taskLoader.PoseDataset));
        }

        private static string DatasetToDropdownValue(PoseDataset dataset)
        {
            switch (dataset)
            {
                case PoseDataset.TaskAPlane:
                    return PoseDatasetChoices[0];
                case PoseDataset.TaskBCylinder:
                    return PoseDatasetChoices[1];
                case PoseDataset.TaskCCoordinateTransform:
                    return PoseDatasetChoices[2];
                default:
                    return PoseDatasetChoices[0];
            }
        }

        private static PoseDataset DropdownValueToDataset(string value)
        {
            if (string.Equals(value, PoseDatasetChoices[1], StringComparison.Ordinal))
            {
                return PoseDataset.TaskBCylinder;
            }

            if (string.Equals(value, PoseDatasetChoices[2], StringComparison.Ordinal))
            {
                return PoseDataset.TaskCCoordinateTransform;
            }

            return PoseDataset.TaskAPlane;
        }

        private void PlaceRobotBaseForCurrentTask()
        {
            if (robotVisualizer == null || _taskData == null || _taskData.Poses.Count == 0)
            {
                return;
            }

            var referenceTransform = GetBaseReferenceForCurrentDataset();
            if (referenceTransform != null)
            {
                robotVisualizer.SetRobotBasePose(referenceTransform.position, referenceTransform.rotation);
                return;
            }

            var anchor = _taskData.Poses[0].WorldPosition;
            var offset = GetBaseOffsetForCurrentDataset();
            var basePosition = anchor + offset;

            var lookDirection = anchor - basePosition;
            lookDirection.y = 0f;
            var baseRotation = lookDirection.sqrMagnitude > 0.0001f
                ? Quaternion.LookRotation(lookDirection.normalized, Vector3.up)
                : Quaternion.identity;

            robotVisualizer.SetRobotBasePose(basePosition, baseRotation);
        }

        private Transform GetBaseReferenceForCurrentDataset()
        {
            if (taskLoader == null)
            {
                return null;
            }

            switch (taskLoader.PoseDataset)
            {
                case PoseDataset.TaskAPlane:
                    return taskABaseReference;
                case PoseDataset.TaskBCylinder:
                    return taskBBaseReference;
                case PoseDataset.TaskCCoordinateTransform:
                    return taskCBaseReference;
                default:
                    return null;
            }
        }

        private Quaternion GetMarkerRotationOffsetForCurrentDataset()
        {
            if (taskLoader == null)
            {
                return Quaternion.identity;
            }

            switch (taskLoader.PoseDataset)
            {
                case PoseDataset.TaskBCylinder:
                case PoseDataset.TaskCCoordinateTransform:
                    return Quaternion.Euler(0f, 90f, 0f);
                default:
                    return Quaternion.identity;
            }
        }

        private Vector3 GetBaseOffsetForCurrentDataset()
        {
            if (taskLoader == null)
            {
                return taskAPlaneRobotBaseOffset;
            }

            switch (taskLoader.PoseDataset)
            {
                case PoseDataset.TaskAPlane:
                    return taskAPlaneRobotBaseOffset;
                case PoseDataset.TaskBCylinder:
                    return taskBCylinderRobotBaseOffset;
                case PoseDataset.TaskCCoordinateTransform:
                    return taskCCoordinateRobotBaseOffset;
                default:
                    return taskAPlaneRobotBaseOffset;
            }
        }
    }
}
