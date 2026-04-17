using System.Collections.Generic;
using UnityEngine;

namespace RobotArmSimulator
{
    public sealed class SimpleJointRobotVisualizer : MonoBehaviour
    {
        [Header("Hierarchy")]
        [SerializeField] private Transform robotRoot;
        [SerializeField] private bool autoCreateHierarchyWhenMissing = true;
        [SerializeField] private float linkLength = 0.12f;
        [SerializeField] private float linkThickness = 0.03f;
        [SerializeField] private Color linkColor = new Color(0.78f, 0.83f, 0.92f, 1f);
        [SerializeField] private Color jointColor = new Color(0.16f, 0.58f, 0.98f, 1f);
        [SerializeField] private Color toolMarkerColor = new Color(1f, 0.52f, 0.18f, 1f);
        [SerializeField] private Color ikTargetColor = new Color(1f, 0.94f, 0.2f, 0.9f);
        [SerializeField] private bool showIkTargetMarker;

        [Header("Joint Axes")]
        [SerializeField] private Vector3 joint1Axis = Vector3.up;
        [SerializeField] private Vector3 joint2Axis = Vector3.right;
        [SerializeField] private Vector3 joint3Axis = Vector3.right;
        [SerializeField] private Vector3 joint4Axis = Vector3.forward;
        [SerializeField] private Vector3 joint5Axis = Vector3.right;
        [SerializeField] private Vector3 joint6Axis = Vector3.forward;

        private readonly List<Transform> _jointPivots = new List<Transform>(6);
        private readonly float[] _currentJointAnglesDeg = new float[6];
        private Transform _toolMarkerTransform;
        private Transform _toolTargetTransform;

        public int JointCount
        {
            get
            {
                EnsureHierarchy();
                return _jointPivots.Count;
            }
        }

        public Transform ToolTipTransform
        {
            get
            {
                EnsureHierarchy();
                return _toolMarkerTransform;
            }
        }

        public Transform ToolMarkerTransform
        {
            get
            {
                EnsureHierarchy();
                return _toolMarkerTransform;
            }
        }

        public Transform ToolTargetTransform
        {
            get
            {
                EnsureHierarchy();
                return _toolTargetTransform;
            }
        }

        private void Awake()
        {
            EnsureHierarchy();
        }

        public void ApplyJointAngles(IReadOnlyList<float> jointAnglesDeg)
        {
            EnsureHierarchy();
            if (_jointPivots.Count == 0)
            {
                return;
            }

            for (var i = 0; i < _jointPivots.Count; i++)
            {
                var pivot = _jointPivots[i];
                if (pivot == null)
                {
                    continue;
                }

                var angle = jointAnglesDeg != null && i < jointAnglesDeg.Count ? jointAnglesDeg[i] : 0f;
                var axis = GetJointAxis(i);
                if (axis.sqrMagnitude < 0.0001f)
                {
                    axis = Vector3.up;
                }

                pivot.localRotation = Quaternion.AngleAxis(angle, axis.normalized);
                _currentJointAnglesDeg[i] = NormalizeAngle(angle);
            }

            // Keep IK target aligned with current marker when explicit joint values are applied.
            if (_toolTargetTransform != null && _toolMarkerTransform != null)
            {
                _toolTargetTransform.SetPositionAndRotation(_toolMarkerTransform.position, _toolMarkerTransform.rotation);
            }
        }

        public bool TryGetJoint(int index, out Transform pivot, out Vector3 localAxis)
        {
            EnsureHierarchy();
            if (index < 0 || index >= _jointPivots.Count)
            {
                pivot = null;
                localAxis = Vector3.up;
                return false;
            }

            pivot = _jointPivots[index];
            localAxis = GetJointAxis(index);
            if (localAxis.sqrMagnitude < 0.0001f)
            {
                localAxis = Vector3.up;
            }

            return pivot != null;
        }

        public void ApplyWorldDeltaToJoint(int index, float deltaDeg)
        {
            if (!TryGetJoint(index, out var pivot, out var localAxis))
            {
                return;
            }

            var axisWorld = pivot.TransformDirection(localAxis.normalized);
            if (axisWorld.sqrMagnitude < 0.000001f)
            {
                return;
            }

            pivot.rotation = Quaternion.AngleAxis(deltaDeg, axisWorld.normalized) * pivot.rotation;
            _currentJointAnglesDeg[index] = NormalizeAngle(_currentJointAnglesDeg[index] + deltaDeg);
        }

        public void SetToolMarkerPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            SetToolTargetPose(worldPosition, worldRotation);
        }

        public void SetToolTargetPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            EnsureHierarchy();
            if (_toolTargetTransform == null)
            {
                return;
            }

            _toolTargetTransform.SetPositionAndRotation(worldPosition, worldRotation);
        }

        public float[] GetCurrentJointAnglesCopy()
        {
            var copy = new float[_currentJointAnglesDeg.Length];
            for (var i = 0; i < copy.Length; i++)
            {
                copy[i] = _currentJointAnglesDeg[i];
            }

            return copy;
        }

        public void ResetToHomePose()
        {
            ApplyJointAngles(new float[6]);
        }

        public void SetRobotBasePose(Vector3 worldPosition, Quaternion worldRotation)
        {
            EnsureHierarchy();
            if (robotRoot == null)
            {
                return;
            }

            robotRoot.SetPositionAndRotation(worldPosition, worldRotation);
        }

        private void EnsureHierarchy()
        {
            if (robotRoot == null)
            {
                if (!autoCreateHierarchyWhenMissing)
                {
                    return;
                }

                var rootObject = new GameObject("Simple6AxisRobot");
                rootObject.transform.SetParent(transform, false);
                robotRoot = rootObject.transform;
            }

            if (_jointPivots.Count == 6)
            {
                return;
            }

            _jointPivots.Clear();
            BuildHierarchy();
        }

        private void BuildHierarchy()
        {
            Transform parent = robotRoot;
            for (var i = 0; i < 6; i++)
            {
                var pivot = new GameObject($"Joint{i + 1}_Pivot").transform;
                pivot.SetParent(parent, false);
                pivot.localPosition = i == 0 ? Vector3.zero : new Vector3(0f, linkLength, 0f);
                pivot.localRotation = Quaternion.identity;
                _jointPivots.Add(pivot);

                CreateJointVisual(pivot, i);
                CreateLinkVisual(pivot, i);
                parent = pivot;
            }

            var toolMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            toolMarker.name = "ToolMarker";
            toolMarker.transform.SetParent(parent, false);
            toolMarker.transform.localPosition = new Vector3(0f, linkLength, 0f);
            toolMarker.transform.localScale = Vector3.one * Mathf.Max(0.01f, linkThickness * 0.9f);
            ApplyMaterialColor(toolMarker.GetComponent<Renderer>(), toolMarkerColor);
            RemoveCollider(toolMarker);
            _toolMarkerTransform = toolMarker.transform;

            var ikTarget = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ikTarget.name = "IkTargetMarker";
            ikTarget.transform.SetParent(robotRoot, false);
            ikTarget.transform.position = _toolMarkerTransform.position;
            ikTarget.transform.rotation = _toolMarkerTransform.rotation;
            ikTarget.transform.localScale = Vector3.one * Mathf.Max(0.012f, linkThickness * 0.72f);
            var ikTargetRenderer = ikTarget.GetComponent<Renderer>();
            ApplyMaterialColor(ikTargetRenderer, ikTargetColor);
            if (ikTargetRenderer != null)
            {
                ikTargetRenderer.enabled = showIkTargetMarker;
            }

            RemoveCollider(ikTarget);
            _toolTargetTransform = ikTarget.transform;

            for (var i = 0; i < _currentJointAnglesDeg.Length; i++)
            {
                _currentJointAnglesDeg[i] = 0f;
            }
        }

        private void CreateJointVisual(Transform parent, int index)
        {
            var joint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            joint.name = $"Joint{index + 1}_Visual";
            joint.transform.SetParent(parent, false);
            joint.transform.localPosition = Vector3.zero;
            joint.transform.localScale = Vector3.one * Mathf.Max(0.01f, linkThickness * 1.1f);
            ApplyMaterialColor(joint.GetComponent<Renderer>(), jointColor);
            RemoveCollider(joint);
        }

        private void CreateLinkVisual(Transform parent, int index)
        {
            var link = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            link.name = $"Link{index + 1}_Visual";
            link.transform.SetParent(parent, false);
            link.transform.localPosition = new Vector3(0f, linkLength * 0.5f, 0f);
            link.transform.localRotation = Quaternion.identity;
            link.transform.localScale = new Vector3(
                Mathf.Max(0.01f, linkThickness),
                Mathf.Max(0.01f, linkLength * 0.5f),
                Mathf.Max(0.01f, linkThickness));
            ApplyMaterialColor(link.GetComponent<Renderer>(), linkColor);
            RemoveCollider(link);
        }

        private Vector3 GetJointAxis(int index)
        {
            switch (index)
            {
                case 0:
                    return joint1Axis;
                case 1:
                    return joint2Axis;
                case 2:
                    return joint3Axis;
                case 3:
                    return joint4Axis;
                case 4:
                    return joint5Axis;
                case 5:
                    return joint6Axis;
                default:
                    return Vector3.up;
            }
        }

        private static void ApplyMaterialColor(Renderer renderer, Color color)
        {
            if (renderer == null)
            {
                return;
            }

            var material = new Material(renderer.sharedMaterial);
            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }
            else
            {
                material.color = color;
            }

            renderer.material = material;
        }

        private static void RemoveCollider(GameObject obj)
        {
            if (obj == null)
            {
                return;
            }

            var collider = obj.GetComponent<Collider>();
            if (collider == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(collider);
            }
            else
            {
                DestroyImmediate(collider);
            }
        }

        private static float NormalizeAngle(float angle)
        {
            return Mathf.Repeat(angle + 180f, 360f) - 180f;
        }
    }
}
