using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotArmSimulator
{
    public enum JointTrajectoryPlaybackState
    {
        Stopped,
        Playing,
        Paused,
        Completed
    }

    public enum JointTrajectoryPlaybackMode
    {
        WaypointIk,
        JointCsv
    }

    public sealed class JointTrajectoryFrameData
    {
        public JointTrajectoryFrameData(
            float timeSec,
            float[] jointAnglesDeg,
            string segment,
            int waypointIndex,
            Vector3 targetPosition,
            Quaternion targetRotation,
            bool hasTargetPose)
        {
            TimeSec = Mathf.Max(0f, timeSec);
            JointAnglesDeg = new float[6];
            var hasJointValues = false;
            for (var i = 0; i < JointAnglesDeg.Length; i++)
            {
                JointAnglesDeg[i] = jointAnglesDeg != null && i < jointAnglesDeg.Length ? jointAnglesDeg[i] : 0f;
                hasJointValues = hasJointValues || (jointAnglesDeg != null && i < jointAnglesDeg.Length);
            }

            Segment = string.IsNullOrWhiteSpace(segment) ? "-" : segment;
            WaypointIndex = waypointIndex;
            TargetPosition = targetPosition;
            TargetRotation = targetRotation;
            HasTargetPose = hasTargetPose;
            HasJointValues = hasJointValues;
        }

        public float TimeSec { get; }
        public float[] JointAnglesDeg { get; }
        public string Segment { get; }
        public int WaypointIndex { get; }
        public Vector3 TargetPosition { get; }
        public Quaternion TargetRotation { get; }
        public bool HasTargetPose { get; }
        public bool HasJointValues { get; }
    }

    public sealed class JointTrajectoryPlaybackController : MonoBehaviour
    {
        [Header("Playback")]
        [SerializeField] private float playbackSpeed = 1f;
        [SerializeField] private bool loopPlayback;
        [SerializeField] private bool snapToStartOnStop = true;

        [Header("Mode")]
        [SerializeField] private JointTrajectoryPlaybackMode playbackMode = JointTrajectoryPlaybackMode.WaypointIk;
        [SerializeField] private bool useTrajectoryDurationForWaypoints = true;
        [SerializeField] private float waypointSecondsPerSegment = 0.6f;

        public event Action<JointTrajectoryFrameData> OnFrameUpdated;
        public event Action<JointTrajectoryPlaybackState> OnPlaybackStateChanged;
        public event Action OnPlaybackCompleted;

        private readonly List<JointTrajectorySampleData> _samples = new List<JointTrajectorySampleData>();
        private readonly List<PoseData> _waypoints = new List<PoseData>();
        private JointTrajectoryPlaybackState _state = JointTrajectoryPlaybackState.Stopped;
        private float _currentTimeSec;
        private int _lastLowerIndex;

        public JointTrajectoryPlaybackState State => _state;
        public JointTrajectoryPlaybackMode PlaybackMode => playbackMode;
        public JointTrajectoryFrameData CurrentFrame { get; private set; }
        public bool HasPlayableData => IsDataReadyForPlayback();
        public float CurrentTimeSec => _currentTimeSec;
        public float StartTimeSec => GetStartTimeForMode();
        public float DurationSec => GetEndTimeForMode();

        private void Update()
        {
            if (_state != JointTrajectoryPlaybackState.Playing || !IsDataReadyForPlayback())
            {
                return;
            }

            _currentTimeSec += Time.deltaTime * Mathf.Max(0.01f, playbackSpeed);

            var startTime = StartTimeSec;
            var endTime = DurationSec;
            if (_currentTimeSec >= endTime)
            {
                if (loopPlayback && endTime > startTime + 0.0001f)
                {
                    _currentTimeSec = startTime + Mathf.Repeat(_currentTimeSec - startTime, endTime - startTime);
                    EvaluateAndNotify();
                    return;
                }

                _currentTimeSec = endTime;
                EvaluateAndNotify();
                SetState(JointTrajectoryPlaybackState.Completed);
                OnPlaybackCompleted?.Invoke();
                return;
            }

            EvaluateAndNotify();
        }

        public void SetData(
            IReadOnlyList<JointTrajectorySampleData> samples,
            IReadOnlyList<PoseData> waypoints)
        {
            _samples.Clear();
            if (samples != null)
            {
                for (var i = 0; i < samples.Count; i++)
                {
                    var sample = samples[i];
                    if (sample != null)
                    {
                        _samples.Add(sample);
                    }
                }
            }

            _samples.Sort((a, b) => a.TimeSec.CompareTo(b.TimeSec));

            _waypoints.Clear();
            if (waypoints != null)
            {
                for (var i = 0; i < waypoints.Count; i++)
                {
                    var pose = waypoints[i];
                    if (pose != null)
                    {
                        _waypoints.Add(pose);
                    }
                }
            }

            ResetPlaybackToStart();
        }

        public void SetSamples(IReadOnlyList<JointTrajectorySampleData> samples)
        {
            var waypointsSnapshot = _waypoints.Count > 0 ? new List<PoseData>(_waypoints) : null;
            SetData(samples, waypointsSnapshot);
        }

        public void SetWaypoints(IReadOnlyList<PoseData> waypoints)
        {
            var samplesSnapshot = _samples.Count > 0 ? new List<JointTrajectorySampleData>(_samples) : null;
            SetData(samplesSnapshot, waypoints);
        }

        public void Play()
        {
            if (!IsDataReadyForPlayback())
            {
                return;
            }

            if (_state == JointTrajectoryPlaybackState.Completed)
            {
                ResetPlaybackToStart();
            }

            SetState(JointTrajectoryPlaybackState.Playing);
        }

        public void Pause()
        {
            if (_state == JointTrajectoryPlaybackState.Playing)
            {
                SetState(JointTrajectoryPlaybackState.Paused);
            }
        }

        public void Stop()
        {
            if (IsDataReadyForPlayback() && snapToStartOnStop)
            {
                ResetPlaybackToStart();
            }

            SetState(JointTrajectoryPlaybackState.Stopped);
        }

        public void SetPlaybackMode(JointTrajectoryPlaybackMode mode)
        {
            if (playbackMode == mode) return;
            Stop();
            playbackMode = mode;
            ResetPlaybackToStart();
        }

        private void EvaluateAndNotify()
        {
            if (!IsDataReadyForPlayback())
            {
                CurrentFrame = null;
                return;
            }

            CurrentFrame = EvaluateFrameAtTime(_currentTimeSec);
            OnFrameUpdated?.Invoke(CurrentFrame);
        }

        private JointTrajectoryFrameData EvaluateFrameAtTime(float timeSec)
        {
            return playbackMode == JointTrajectoryPlaybackMode.WaypointIk
                ? EvaluateWaypointFrameAtTime(timeSec)
                : EvaluateJointFrameAtTime(timeSec);
        }

        private JointTrajectoryFrameData EvaluateJointFrameAtTime(float timeSec)
        {
            if (_samples.Count == 0)
            {
                return null;
            }

            if (_samples.Count == 1)
            {
                var only = _samples[0];
                return new JointTrajectoryFrameData(
                    only.TimeSec,
                    only.JointAnglesDeg,
                    only.Segment,
                    -1,
                    Vector3.zero,
                    Quaternion.identity,
                    false);
            }

            var sampleStartTime = _samples[0].TimeSec;
            var sampleEndTime = _samples[_samples.Count - 1].TimeSec;
            var clampedTime = Mathf.Clamp(timeSec, sampleStartTime, sampleEndTime);
            var lower = Mathf.Clamp(_lastLowerIndex, 0, _samples.Count - 2);

            if (clampedTime < _samples[lower].TimeSec)
            {
                lower = 0;
            }

            while (lower < _samples.Count - 2 && _samples[lower + 1].TimeSec <= clampedTime)
            {
                lower++;
            }

            while (lower > 0 && _samples[lower].TimeSec > clampedTime)
            {
                lower--;
            }

            _lastLowerIndex = lower;
            var upper = Mathf.Min(lower + 1, _samples.Count - 1);

            var sampleA = _samples[lower];
            var sampleB = _samples[upper];
            var blend = 0f;
            var span = sampleB.TimeSec - sampleA.TimeSec;
            if (span > 0.0001f)
            {
                blend = Mathf.Clamp01((clampedTime - sampleA.TimeSec) / span);
            }

            var joints = new float[6];
            for (var i = 0; i < joints.Length; i++)
            {
                var a = sampleA.JointAnglesDeg != null && i < sampleA.JointAnglesDeg.Length ? sampleA.JointAnglesDeg[i] : 0f;
                var b = sampleB.JointAnglesDeg != null && i < sampleB.JointAnglesDeg.Length ? sampleB.JointAnglesDeg[i] : 0f;
                joints[i] = Mathf.Lerp(a, b, blend);
            }

            var segment = blend < 0.5f ? sampleA.Segment : sampleB.Segment;
            return new JointTrajectoryFrameData(
                clampedTime,
                joints,
                segment,
                -1,
                Vector3.zero,
                Quaternion.identity,
                false);
        }

        private JointTrajectoryFrameData EvaluateWaypointFrameAtTime(float timeSec)
        {
            if (_waypoints.Count == 0)
            {
                return null;
            }

            var startTime = StartTimeSec;
            var endTime = DurationSec;
            var clampedTime = Mathf.Clamp(timeSec, startTime, endTime);

            if (_waypoints.Count == 1)
            {
                var only = _waypoints[0];
                var joints = EvaluateJointAnglesAtTime(clampedTime, out var segment);
                if (string.IsNullOrWhiteSpace(segment) || segment == "-")
                {
                    segment = "waypoint";
                }

                return new JointTrajectoryFrameData(
                    clampedTime,
                    joints,
                    segment,
                    0,
                    only.WorldPosition,
                    only.WorldRotation,
                    true);
            }

            var duration = Mathf.Max(0.0001f, endTime - startTime);
            var normalized = Mathf.Clamp01((clampedTime - startTime) / duration);
            var scaled = normalized * (_waypoints.Count - 1);
            var lower = Mathf.Clamp(Mathf.FloorToInt(scaled), 0, _waypoints.Count - 2);
            var upper = Mathf.Min(lower + 1, _waypoints.Count - 1);
            var blend = Mathf.Clamp01(scaled - lower);

            var poseA = _waypoints[lower];
            var poseB = _waypoints[upper];
            var targetPosition = Vector3.Lerp(poseA.WorldPosition, poseB.WorldPosition, blend);
            var targetRotation = Quaternion.Slerp(poseA.WorldRotation, poseB.WorldRotation, blend);

            var jointsForUi = EvaluateJointAnglesAtTime(clampedTime, out var segmentLabel);
            if (string.IsNullOrWhiteSpace(segmentLabel) || segmentLabel == "-")
            {
                segmentLabel = $"wp {lower:00}->{upper:00}";
            }

            var waypointIndex = blend < 0.5f ? lower : upper;
            return new JointTrajectoryFrameData(
                clampedTime,
                jointsForUi,
                segmentLabel,
                waypointIndex,
                targetPosition,
                targetRotation,
                true);
        }

        private float[] EvaluateJointAnglesAtTime(float timeSec, out string segment)
        {
            segment = "-";
            if (_samples.Count == 0)
            {
                return null;
            }

            var frame = EvaluateJointFrameAtTime(timeSec);
            if (frame == null)
            {
                return null;
            }

            segment = frame.Segment;
            return frame.JointAnglesDeg;
        }

        private bool IsDataReadyForPlayback()
        {
            return playbackMode == JointTrajectoryPlaybackMode.WaypointIk
                ? _waypoints.Count > 0
                : _samples.Count > 0;
        }

        private float GetStartTimeForMode()
        {
            if (playbackMode == JointTrajectoryPlaybackMode.JointCsv)
            {
                return _samples.Count > 0 ? _samples[0].TimeSec : 0f;
            }

            if (useTrajectoryDurationForWaypoints && _samples.Count > 0)
            {
                return _samples[0].TimeSec;
            }

            return 0f;
        }

        private float GetEndTimeForMode()
        {
            var start = GetStartTimeForMode();
            if (playbackMode == JointTrajectoryPlaybackMode.JointCsv)
            {
                return _samples.Count > 0 ? _samples[_samples.Count - 1].TimeSec : start;
            }

            return start + GetWaypointPlaybackDuration();
        }

        private float GetWaypointPlaybackDuration()
        {
            if (_waypoints.Count <= 1)
            {
                return Mathf.Max(0.1f, waypointSecondsPerSegment);
            }

            if (useTrajectoryDurationForWaypoints && _samples.Count > 1)
            {
                var durationFromCsv = _samples[_samples.Count - 1].TimeSec - _samples[0].TimeSec;
                return Mathf.Max(0.1f, durationFromCsv);
            }

            return Mathf.Max(0.1f, waypointSecondsPerSegment * (_waypoints.Count - 1));
        }

        private void ResetPlaybackToStart()
        {
            _lastLowerIndex = 0;
            _currentTimeSec = StartTimeSec;
            EvaluateAndNotify();
            SetState(JointTrajectoryPlaybackState.Stopped);
        }

        private void SetState(JointTrajectoryPlaybackState next)
        {
            if (_state == next)
            {
                return;
            }

            _state = next;
            OnPlaybackStateChanged?.Invoke(_state);
        }
    }
}
