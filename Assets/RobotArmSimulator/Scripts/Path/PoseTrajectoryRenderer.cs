using System.Collections.Generic;
using UnityEngine;

namespace RobotArmSimulator
{
    public sealed class PoseTrajectoryRenderer : MonoBehaviour
    {
        [Header("Waypoint Marker")]
        [SerializeField] private PrimitiveType markerShape = PrimitiveType.Sphere;
        [SerializeField] private float markerScale = 0.028f;
        [SerializeField] private float selectedMarkerScale = 0.045f;
        [SerializeField] private Color markerColor = new Color(0.17f, 0.78f, 1f, 1f);
        [SerializeField] private Color selectedMarkerColor = new Color(1f, 0.9f, 0.2f, 1f);
        [SerializeField] private bool showWaypointLabels = true;

        [Header("Path")]
        [SerializeField] private LineRenderer pathLine;
        [SerializeField] private float pathWidth = 0.006f;
        [SerializeField] private Color pathColor = new Color(0.2f, 0.92f, 1f, 0.95f);

        [Header("Orientation Visual")]
        [SerializeField] private bool showOrientationAxes = true;
        [SerializeField] private float axisLength = 0.045f;
        [SerializeField] private float axisWidth = 0.0028f;
        [SerializeField] private float forwardArrowLength = 0.06f;
        [SerializeField] private Color forwardArrowColor = new Color(1f, 0.62f, 0.15f, 1f);

        private sealed class MarkerVisual
        {
            public Transform Transform;
            public Renderer Renderer;
            public Color BaseColor;
            public Vector3 BaseScale;
        }

        private readonly List<MarkerVisual> _markerVisuals = new List<MarkerVisual>();
        private GameObject _runtimeRoot;
        private int _selectedIndex = -1;
        private Quaternion _markerRotationOffset = Quaternion.identity;

        public void SetMarkerRotationOffset(Quaternion offset)
        {
            _markerRotationOffset = offset;
        }

        public int Count => _markerVisuals.Count;
        public int SelectedIndex => _selectedIndex;

        private void Awake()
        {
            EnsurePathLine();
        }

        public void Render(IReadOnlyList<PoseData> poses)
        {
            Clear();
            EnsurePathLine();

            if (poses == null || poses.Count == 0)
            {
                pathLine.positionCount = 0;
                return;
            }

            _runtimeRoot = new GameObject("PoseMarkers");
            _runtimeRoot.transform.SetParent(transform, false);

            var points = new Vector3[poses.Count];
            for (var i = 0; i < poses.Count; i++)
            {
                var pose = poses[i];
                if (pose == null)
                {
                    continue;
                }

                points[i] = pose.WorldPosition;
                var markerVisual = BuildMarker(pose, i);
                _markerVisuals.Add(markerVisual);
            }

            pathLine.positionCount = points.Length;
            pathLine.SetPositions(points);
            Highlight(0);
        }

        public void Highlight(int index)
        {
            if (_selectedIndex >= 0 && _selectedIndex < _markerVisuals.Count)
            {
                var previous = _markerVisuals[_selectedIndex];
                ApplyState(previous, previous.BaseColor, previous.BaseScale);
            }

            if (index < 0 || index >= _markerVisuals.Count)
            {
                _selectedIndex = -1;
                return;
            }

            _selectedIndex = index;
            var selected = _markerVisuals[index];
            ApplyState(selected, selectedMarkerColor, Vector3.one * selectedMarkerScale);
        }

        public void Clear()
        {
            _selectedIndex = -1;
            _markerVisuals.Clear();

            if (_runtimeRoot != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(_runtimeRoot);
                }
                else
                {
                    DestroyImmediate(_runtimeRoot);
                }
            }

            _runtimeRoot = null;
            if (pathLine != null)
            {
                pathLine.positionCount = 0;
            }
        }

        private MarkerVisual BuildMarker(PoseData pose, int index)
        {
            var markerObject = GameObject.CreatePrimitive(markerShape);
            markerObject.name = $"Pose_{index:000}";
            markerObject.transform.SetParent(_runtimeRoot.transform, false);
            markerObject.transform.position = pose.WorldPosition;
            markerObject.transform.rotation = pose.WorldRotation * _markerRotationOffset;
            markerObject.transform.localScale = Vector3.one * markerScale;

            var collider = markerObject.GetComponent<Collider>();
            if (collider != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(collider);
                }
                else
                {
                    DestroyImmediate(collider);
                }
            }

            var renderer = markerObject.GetComponent<Renderer>();
            if (renderer != null)
            {
                var material = new Material(renderer.sharedMaterial);
                ApplyColor(material, markerColor);
                renderer.material = material;
            }

            if (showWaypointLabels)
            {
                CreateLabel(markerObject.transform, pose.Id);
            }

            if (showOrientationAxes)
            {
                CreateOrientationAxes(markerObject.transform);
            }

            return new MarkerVisual
            {
                Transform = markerObject.transform,
                Renderer = renderer,
                BaseColor = markerColor,
                BaseScale = Vector3.one * markerScale
            };
        }

        private void CreateLabel(Transform parent, string text)
        {
            var labelObject = new GameObject("PoseLabel");
            labelObject.transform.SetParent(parent, false);
            labelObject.transform.localPosition = new Vector3(0f, markerScale * 2.2f, 0f);

            var textMesh = labelObject.AddComponent<TextMesh>();
            textMesh.text = text;
            textMesh.fontSize = 48;
            textMesh.characterSize = 0.014f;
            textMesh.anchor = TextAnchor.MiddleCenter;
            textMesh.alignment = TextAlignment.Center;
            textMesh.color = Color.black;
        }

        private void CreateOrientationAxes(Transform parent)
        {
            var axesRoot = new GameObject("Axes");
            axesRoot.transform.SetParent(parent, false);

            CreateAxisLine(axesRoot.transform, "AxisX", Color.red, Vector3.right * axisLength, axisWidth);
            CreateAxisLine(axesRoot.transform, "AxisY", Color.green, Vector3.up * axisLength, axisWidth);
            CreateAxisLine(axesRoot.transform, "AxisZ", Color.blue, Vector3.forward * axisLength, axisWidth);
            CreateAxisLine(axesRoot.transform, "ForwardArrow", forwardArrowColor, Vector3.forward * forwardArrowLength, axisWidth * 1.1f);
        }

        private void CreateAxisLine(Transform parent, string lineName, Color color, Vector3 endPosition, float width)
        {
            var lineObject = new GameObject(lineName);
            lineObject.transform.SetParent(parent, false);
            var line = lineObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.positionCount = 2;
            line.startWidth = width;
            line.endWidth = width;
            line.startColor = color;
            line.endColor = color;
            line.material = CreateDefaultLineMaterial();
            line.SetPosition(0, Vector3.zero);
            line.SetPosition(1, endPosition);
        }

        private void EnsurePathLine()
        {
            if (pathLine == null)
            {
                pathLine = GetComponent<LineRenderer>();
                if (pathLine == null)
                {
                    pathLine = gameObject.AddComponent<LineRenderer>();
                }
            }

            pathLine.useWorldSpace = true;
            pathLine.startWidth = pathWidth;
            pathLine.endWidth = pathWidth;
            pathLine.startColor = pathColor;
            pathLine.endColor = pathColor;
            pathLine.material = CreateDefaultLineMaterial();
            pathLine.positionCount = 0;
        }

        private static void ApplyState(MarkerVisual visual, Color color, Vector3 scale)
        {
            if (visual == null)
            {
                return;
            }

            if (visual.Transform != null)
            {
                visual.Transform.localScale = scale;
            }

            if (visual.Renderer != null)
            {
                ApplyColor(visual.Renderer.material, color);
            }
        }

        private static void ApplyColor(Material material, Color color)
        {
            if (material == null)
            {
                return;
            }

            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }
            else if (material.HasProperty("_Color"))
            {
                material.color = color;
            }
        }

        private static Material CreateDefaultLineMaterial()
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit")
                         ?? Shader.Find("Standard")
                         ?? Shader.Find("Sprites/Default");
            return shader != null ? new Material(shader) : null;
        }
    }
}
