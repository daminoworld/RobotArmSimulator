using UnityEngine;

namespace RobotArmSimulator
{
    /// <summary>
    /// Damped Least Squares (DLS) Jacobian IK Solver.
    ///
    /// Δθ = Jᵀ (JJᵀ + λ²I)⁻¹ Δx
    ///
    /// J (3×n): position Jacobian — column i = axis_i × (tip − pivot_i)
    /// λ (dampingFactor): suppresses joint velocity near singularities.
    ///   Higher λ → smoother / more stable, but slower convergence.
    ///
    /// Unlike CCD (per-joint clamp), DLS scales Δθ uniformly so the
    /// optimal joint-ratio computed by the pseudo-inverse is preserved.
    /// </summary>
    public sealed class JacobianDlsIkSolver : MonoBehaviour, IIkSolver
    {
        [Header("References")]
        [SerializeField] private SimpleJointRobotVisualizer robotVisualizer;

        [Header("IK")]
        [SerializeField] private bool solveInLateUpdate = true;
        [SerializeField] private int iterationsPerFrame = 10;
        [SerializeField] private float maxDegreesPerSecond = 220f;
        [SerializeField] private float positionTolerance = 0.0025f;
        [SerializeField] private float dampingFactor = 0.05f;

        [Header("Orientation")]
        [SerializeField] private bool alignOrientation = true;
        [SerializeField, Range(0f, 1f)] private float orientationBlend = 0.35f;
        [SerializeField] private int wristJointCount = 3;

        // Pre-allocated working buffers — avoids per-frame GC allocations.
        // Sized for n = 6 joints (fixed for this robot).
        private readonly float[,] _J      = new float[3, 6];
        private readonly float[,] _JJT    = new float[3, 3];
        private readonly float[,] _inv    = new float[3, 3];
        private readonly float[]  _tmp    = new float[3];
        private readonly float[]  _dtheta = new float[6];

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
            var tipPosition    = robotVisualizer.ToolMarkerTransform.position;

            if ((tipPosition - targetPosition).sqrMagnitude <= positionTolerance * positionTolerance)
            {
                return;
            }

            var n         = robotVisualizer.JointCount;
            var iterCount = Mathf.Clamp(iterationsPerFrame, 1, 128);
            var maxStepPerIter = Mathf.Max(0.01f, maxDegreesPerSecond)
                               * Mathf.Max(0.0001f, deltaTime)
                               / iterCount;

            for (var iter = 0; iter < iterCount; iter++)
            {
                tipPosition = robotVisualizer.ToolMarkerTransform.position;
                var error = targetPosition - tipPosition;

                if (error.sqrMagnitude <= positionTolerance * positionTolerance)
                {
                    break;
                }

                // --- Build position Jacobian J (3×n) ---
                // Column i: axis_i × (tip − pivot_i)
                for (var i = 0; i < n; i++)
                {
                    if (!robotVisualizer.TryGetJoint(i, out var pivot, out var localAxis))
                    {
                        _J[0, i] = 0f; _J[1, i] = 0f; _J[2, i] = 0f;
                        continue;
                    }

                    var axisWorld = pivot.TransformDirection(localAxis.normalized);
                    var col       = Vector3.Cross(axisWorld, tipPosition - pivot.position);
                    _J[0, i] = col.x;
                    _J[1, i] = col.y;
                    _J[2, i] = col.z;
                }

                // --- JJᵀ (3×3) ---
                for (var r = 0; r < 3; r++)
                {
                    for (var c = 0; c < 3; c++)
                    {
                        var sum = 0f;
                        for (var k = 0; k < n; k++) sum += _J[r, k] * _J[c, k];
                        _JJT[r, c] = sum;
                    }
                }

                // --- Add damping: JJᵀ + λ²I ---
                var lambda2 = dampingFactor * dampingFactor;
                _JJT[0, 0] += lambda2;
                _JJT[1, 1] += lambda2;
                _JJT[2, 2] += lambda2;

                // --- Invert 3×3 (Cramer's rule) ---
                if (!Invert3x3(_JJT, _inv))
                {
                    continue;
                }

                // --- tmp = inv * error ---
                _tmp[0] = _inv[0, 0] * error.x + _inv[0, 1] * error.y + _inv[0, 2] * error.z;
                _tmp[1] = _inv[1, 0] * error.x + _inv[1, 1] * error.y + _inv[1, 2] * error.z;
                _tmp[2] = _inv[2, 0] * error.x + _inv[2, 1] * error.y + _inv[2, 2] * error.z;

                // --- Δθ = Jᵀ * tmp  (radians) ---
                for (var i = 0; i < n; i++)
                {
                    _dtheta[i] = _J[0, i] * _tmp[0]
                               + _J[1, i] * _tmp[1]
                               + _J[2, i] * _tmp[2];
                }

                // --- Uniform scale to respect maxStepPerIter ---
                // DLS computes an optimal joint-ratio; scaling uniformly preserves it.
                // (Per-joint clamping as in CCD would distort the pseudo-inverse direction.)
                var maxAbsDeg = 0f;
                for (var i = 0; i < n; i++)
                {
                    maxAbsDeg = Mathf.Max(maxAbsDeg, Mathf.Abs(_dtheta[i] * Mathf.Rad2Deg));
                }

                var scale = maxAbsDeg > maxStepPerIter ? maxStepPerIter / maxAbsDeg : 1f;

                for (var i = 0; i < n; i++)
                {
                    var dthetaDeg = _dtheta[i] * Mathf.Rad2Deg * scale;
                    if (Mathf.Abs(dthetaDeg) > 0.0001f)
                    {
                        robotVisualizer.ApplyWorldDeltaToJoint(i, dthetaDeg);
                    }
                }
            }

            if (alignOrientation)
            {
                SolveOrientation(maxDegreesPerSecond
                                 * Mathf.Max(0.0001f, deltaTime)
                                 / Mathf.Max(1, iterationsPerFrame));
            }
        }

        public void SolveImmediately(int iterations = 36)
        {
            if (robotVisualizer == null)
            {
                return;
            }

            var prevIter  = iterationsPerFrame;
            var prevSpeed = maxDegreesPerSecond;
            iterationsPerFrame  = Mathf.Max(1, iterations);
            maxDegreesPerSecond = Mathf.Max(maxDegreesPerSecond, 1440f);
            Solve(1f);
            maxDegreesPerSecond = prevSpeed;
            iterationsPerFrame  = prevIter;
        }

        /// <summary>
        /// Inverts a 3×3 matrix using Cramer's rule.
        /// Writes result into the pre-allocated <paramref name="inv"/> buffer.
        /// Returns false if the matrix is singular.
        /// </summary>
        private static bool Invert3x3(float[,] m, float[,] inv)
        {
            var a = m[0, 0]; var b = m[0, 1]; var c = m[0, 2];
            var d = m[1, 0]; var e = m[1, 1]; var f = m[1, 2];
            var g = m[2, 0]; var h = m[2, 1]; var k = m[2, 2];

            var det = a * (e * k - f * h)
                    - b * (d * k - f * g)
                    + c * (d * h - e * g);

            if (Mathf.Abs(det) < 1e-9f)
            {
                return false;
            }

            var id = 1f / det;
            inv[0, 0] =  (e * k - f * h) * id;
            inv[0, 1] = -(b * k - c * h) * id;
            inv[0, 2] =  (b * f - c * e) * id;
            inv[1, 0] = -(d * k - f * g) * id;
            inv[1, 1] =  (a * k - c * g) * id;
            inv[1, 2] = -(a * f - c * d) * id;
            inv[2, 0] =  (d * h - e * g) * id;
            inv[2, 1] = -(a * h - b * g) * id;
            inv[2, 2] =  (a * e - b * d) * id;
            return true;
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

            var targetAxis      = rawAxis.normalized;
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
                var deltaDeg     = Mathf.Clamp(signedAngle * contribution, -maxStepPerIteration, maxStepPerIteration);
                if (Mathf.Abs(deltaDeg) > 0.0001f)
                {
                    robotVisualizer.ApplyWorldDeltaToJoint(index, deltaDeg);
                }
            }
        }
    }
}
