using UnityEngine;

namespace RobotArmSimulator
{
    public sealed class Simple6AxisIkSolver : MonoBehaviour, IIkSolver
    {
        [Header("References")]
        [SerializeField] private SimpleJointRobotVisualizer robotVisualizer;

        [Header("IK")]
        [SerializeField] private bool solveInLateUpdate = true;
        [SerializeField] private int iterationsPerFrame = 12;
        [SerializeField] private float maxDegreesPerSecond = 220f;
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

            var iterationCount = Mathf.Clamp(iterationsPerFrame, 1, 128);
            var maxStepPerIteration = Mathf.Max(0.01f, maxDegreesPerSecond) * Mathf.Max(0.0001f, deltaTime) / iterationCount;
            var targetPosition = robotVisualizer.ToolTargetTransform.position;

            for (var iter = 0; iter < iterationCount; iter++)
            {
                if ((robotVisualizer.ToolMarkerTransform.position - targetPosition).sqrMagnitude <= positionTolerance * positionTolerance)
                {
                    break;
                }

                for (var jointIndex = robotVisualizer.JointCount - 1; jointIndex >= 0; jointIndex--)
                {
                    SolveJointForPosition(jointIndex, targetPosition, maxStepPerIteration);
                }
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
            var previousSpeed = maxDegreesPerSecond;
            iterationsPerFrame = Mathf.Max(1, iterations);
            maxDegreesPerSecond = Mathf.Max(maxDegreesPerSecond, 1440f);
            Solve(1f);
            maxDegreesPerSecond = previousSpeed;
            iterationsPerFrame = previousIterations;
        }

        private void SolveJointForPosition(int jointIndex, Vector3 targetPosition, float maxStepPerIteration)
        {
            if (!robotVisualizer.TryGetJoint(jointIndex, out var pivot, out var localAxis))
            {
                return;
            }

            var axisWorld = pivot.TransformDirection(localAxis.normalized);
            if (axisWorld.sqrMagnitude < 0.000001f)
            {
                return;
            }

            var jointPosition = pivot.position;
            var toTip = robotVisualizer.ToolMarkerTransform.position - jointPosition;
            var toTarget = targetPosition - jointPosition;
            var projectedTip = Vector3.ProjectOnPlane(toTip, axisWorld);
            var projectedTarget = Vector3.ProjectOnPlane(toTarget, axisWorld);

            if (projectedTip.sqrMagnitude < 0.0000001f || projectedTarget.sqrMagnitude < 0.0000001f)
            {
                return;
            }

            var signedAngle = Vector3.SignedAngle(projectedTip, projectedTarget, axisWorld);
            var clampedDelta = Mathf.Clamp(signedAngle, -maxStepPerIteration, maxStepPerIteration);
            if (Mathf.Abs(clampedDelta) > 0.0001f)
            {
                robotVisualizer.ApplyWorldDeltaToJoint(jointIndex, clampedDelta);
            }
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
