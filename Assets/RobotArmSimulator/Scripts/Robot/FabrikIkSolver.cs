using UnityEngine;

namespace RobotArmSimulator
{
    public sealed class FabrikIkSolver : MonoBehaviour, IIkSolver
    {
        [Header("References")]
        [SerializeField] private SimpleJointRobotVisualizer robotVisualizer;

        [Header("IK")]
        [SerializeField] private bool solveInLateUpdate = true;
        [SerializeField] private int iterationsPerFrame = 12;
        [SerializeField] private float maxDegreesPerStep = 960f;
        [SerializeField] private float positionTolerance = 0.0025f;

        [Header("Orientation")]
        [SerializeField] private bool alignOrientation = true;
        [SerializeField, Range(0f, 1f)] private float orientationBlend = 0.35f;
        [SerializeField] private int wristJointCount = 3;

        private void Awake()
        {
            if (robotVisualizer == null)
            {
                robotVisualizer = GetComponentInChildren<SimpleJointRobotVisualizer>();
            }
        }

        private void LateUpdate()
        {
            if (solveInLateUpdate)
            {
                Solve(Time.deltaTime);
            }
        }

        public void Solve(float deltaTime)
        {
            if (robotVisualizer == null
                || robotVisualizer.ToolTargetTransform == null
                || robotVisualizer.ToolMarkerTransform == null
                || robotVisualizer.JointCount == 0)
            {
                return;
            }

            var targetPosition = robotVisualizer.ToolTargetTransform.position;
            var tipPosition = robotVisualizer.ToolMarkerTransform.position;

            if ((tipPosition - targetPosition).sqrMagnitude <= positionTolerance * positionTolerance)
            {
                return;
            }

            var iterCount = Mathf.Clamp(iterationsPerFrame, 1, 128);
            var maxStepPerIteration = Mathf.Max(0.01f, maxDegreesPerStep) * Mathf.Max(0.0001f, deltaTime) / iterCount;

            var n = robotVisualizer.JointCount; // 6
            var positions = new Vector3[n + 1]; // joint0..joint5 + tip

            // Collect current world positions
            for (var i = 0; i < n; i++)
            {
                robotVisualizer.TryGetJoint(i, out var pivot, out _);
                positions[i] = pivot.position;
            }
            positions[n] = robotVisualizer.ToolMarkerTransform.position;

            // Compute link lengths from live hierarchy
            var lengths = new float[n];
            for (var i = 0; i < n; i++)
            {
                lengths[i] = Vector3.Distance(positions[i], positions[i + 1]);
                if (lengths[i] < 0.0001f) lengths[i] = 0.0001f;
            }

            var rootPos = positions[0];

            // Check total reach
            var totalReach = 0f;
            for (var i = 0; i < n; i++) totalReach += lengths[i];

            if ((rootPos - targetPosition).magnitude > totalReach)
            {
                // Target unreachable: stretch chain toward target
                for (var i = 0; i < n; i++)
                {
                    var dir = (targetPosition - positions[i]).normalized;
                    positions[i + 1] = positions[i] + dir * lengths[i];
                }
                ApplyPositionsThroughAxis(positions, maxStepPerIteration);
                return;
            }

            // FABRIK iterations: position pass → axis-constrained apply → re-snapshot
            for (var iter = 0; iter < iterCount; iter++)
            {
                if ((positions[n] - targetPosition).sqrMagnitude <= positionTolerance * positionTolerance)
                {
                    break;
                }

                // Backward pass: pull tip to target, propagate toward root
                positions[n] = targetPosition;
                for (var i = n - 1; i >= 0; i--)
                {
                    var dir = (positions[i] - positions[i + 1]).normalized;
                    positions[i] = positions[i + 1] + dir * lengths[i];
                }

                // Forward pass: fix root, push toward tip
                positions[0] = rootPos;
                for (var i = 1; i <= n; i++)
                {
                    var dir = (positions[i] - positions[i - 1]).normalized;
                    positions[i] = positions[i - 1] + dir * lengths[i - 1];
                }

                // Apply axis-constrained rotations toward FABRIK-solved positions
                ApplyPositionsThroughAxis(positions, maxStepPerIteration);

                // Re-snapshot live positions for next iteration
                for (var i = 0; i < n; i++)
                {
                    robotVisualizer.TryGetJoint(i, out var pivot, out _);
                    positions[i] = pivot.position;
                }
                positions[n] = robotVisualizer.ToolMarkerTransform.position;
            }

            if (alignOrientation)
            {
                SolveOrientation(maxStepPerIteration);
            }
        }

        public void SolveImmediately(int iterations = 36)
        {
            if (robotVisualizer == null)
            {
                return;
            }

            var previousIterations = iterationsPerFrame;
            var previousStep = maxDegreesPerStep;
            iterationsPerFrame = Mathf.Max(1, iterations);
            maxDegreesPerStep = Mathf.Max(maxDegreesPerStep, 1440f);
            Solve(1f);
            maxDegreesPerStep = previousStep;
            iterationsPerFrame = previousIterations;
        }

        private void ApplyPositionsThroughAxis(Vector3[] positions, float maxStep)
        {
            var n = robotVisualizer.JointCount;

            // Process tip-to-base (same as CCD): tip joints have direct influence on end-effector
            // and positions[n] ≈ targetPosition after FABRIK passes, so tip joints rotate CCD-optimally
            for (var i = n - 1; i >= 0; i--)
            {
                if (!robotVisualizer.TryGetJoint(i, out var pivot, out var localAxis))
                {
                    continue;
                }

                var axisWorld = pivot.TransformDirection(localAxis.normalized);
                if (axisWorld.sqrMagnitude < 0.000001f)
                {
                    continue;
                }

                // Current direction from this joint toward the next (live hierarchy)
                var nextLivePos = i < n - 1
                    ? GetJointPosition(i + 1)
                    : robotVisualizer.ToolMarkerTransform.position;
                var currentDir = nextLivePos - pivot.position;

                // FABRIK-desired direction toward solved next position
                var desiredDir = positions[i + 1] - pivot.position;

                var projCurrent = Vector3.ProjectOnPlane(currentDir, axisWorld);
                var projDesired = Vector3.ProjectOnPlane(desiredDir, axisWorld);

                if (projCurrent.sqrMagnitude < 0.0000001f || projDesired.sqrMagnitude < 0.0000001f)
                {
                    continue;
                }

                var signedAngle = Vector3.SignedAngle(projCurrent, projDesired, axisWorld);
                var clampedDelta = Mathf.Clamp(signedAngle, -maxStep, maxStep);
                if (Mathf.Abs(clampedDelta) > 0.0001f)
                {
                    robotVisualizer.ApplyWorldDeltaToJoint(i, clampedDelta);
                }
            }
        }

        private Vector3 GetJointPosition(int index)
        {
            robotVisualizer.TryGetJoint(index, out var pivot, out _);
            return pivot != null ? pivot.position : Vector3.zero;
        }

        private void SolveOrientation(float maxStepPerIteration)
        {
            var toolMarker = robotVisualizer.ToolMarkerTransform;
            var toolTarget = robotVisualizer.ToolTargetTransform;
            if (toolMarker == null || toolTarget == null)
            {
                return;
            }

            var delta = toolTarget.rotation * Quaternion.Inverse(toolMarker.rotation);
            delta.ToAngleAxis(out var rawAngle, out var rawAxis);
            var signedAngle = Mathf.DeltaAngle(0f, rawAngle) * orientationBlend;
            if (Mathf.Abs(signedAngle) < 0.01f || rawAxis.sqrMagnitude < 0.000001f)
            {
                return;
            }

            var targetAxis = rawAxis.normalized;
            var firstWristIndex = Mathf.Max(0, robotVisualizer.JointCount - Mathf.Max(1, wristJointCount));
            for (var index = robotVisualizer.JointCount - 1; index >= firstWristIndex; index--)
            {
                if (!robotVisualizer.TryGetJoint(index, out var pivot, out var localAxis))
                {
                    continue;
                }

                var axisWorld = pivot.TransformDirection(localAxis.normalized);
                if (axisWorld.sqrMagnitude < 0.000001f)
                {
                    continue;
                }

                var contribution = Vector3.Dot(axisWorld.normalized, targetAxis);
                var deltaDeg = Mathf.Clamp(signedAngle * contribution, -maxStepPerIteration, maxStepPerIteration);
                if (Mathf.Abs(deltaDeg) > 0.0001f)
                {
                    robotVisualizer.ApplyWorldDeltaToJoint(index, deltaDeg);
                }
            }
        }
    }
}
