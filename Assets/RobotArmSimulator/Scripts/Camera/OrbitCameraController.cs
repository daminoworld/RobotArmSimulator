using System;
using UnityEngine;

namespace RobotArmSimulator
{
    public class OrbitCameraController : MonoBehaviour
    {
        [Header("Orbit")]
        [SerializeField] private float orbitSpeed = 200f;
        [SerializeField] private float minVerticalAngle = -80f;
        [SerializeField] private float maxVerticalAngle = 80f;

        [Header("Pan")]
        [SerializeField] private float panSpeed = 0.5f;

        [Header("Zoom")]
        [SerializeField] private float zoomSpeed = 0.1f;
        [SerializeField] private float minDistance = 0.08f;
        [SerializeField] private float maxDistance = 25f;

        [Header("Frame Fit")]
        [SerializeField] private float frameDistanceMultiplier = 0.85f;
        [SerializeField] private float frameMinDistance = 0.25f;

        public Func<bool> IsPointerBlocked;

        private Vector3 _pivot = new Vector3(0f, 1f, 0f);
        private float _distance = 3f;
        private float _yaw = 145f;
        private float _pitch = 24f;

        private void LateUpdate()
        {
            var pointerBlocked = IsPointerBlocked?.Invoke() == true;

            if (!pointerBlocked)
            {
                HandleOrbit();
                HandlePan();
                HandleZoom();
            }

            UpdateTransform();
        }

        public void FrameBounds(Bounds bounds)
        {
            _pivot = bounds.center;
            var requestedDistance = Mathf.Max(bounds.size.magnitude * frameDistanceMultiplier, frameMinDistance);
            _distance = Mathf.Clamp(requestedDistance, minDistance, maxDistance);
            UpdateTransform();
        }

        public void FocusPoint(Vector3 point)
        {
            _pivot = point;
        }

        private void HandleOrbit()
        {
            if (!PointerInput.IsRightPressed())
            {
                return;
            }

            var delta = PointerInput.GetPointerDelta();
            _yaw += delta.x * orbitSpeed * 0.01f;
            _pitch -= delta.y * orbitSpeed * 0.01f;
            _pitch = Mathf.Clamp(_pitch, minVerticalAngle, maxVerticalAngle);
        }

        private void HandlePan()
        {
            if (!PointerInput.IsMiddlePressed())
            {
                return;
            }

            var right = transform.right;
            var up = transform.up;
            var pointerDelta = PointerInput.GetPointerDelta();
            var delta = (-right * pointerDelta.x + -up * pointerDelta.y) * (_distance * panSpeed * 0.01f);
            _pivot += delta;
        }

        private void HandleZoom()
        {
            var scroll = PointerInput.GetScrollY();
            if (Mathf.Abs(scroll) < 0.0001f)
            {
                return;
            }

            _distance *= 1f - scroll * zoomSpeed;
            _distance = Mathf.Clamp(_distance, minDistance, maxDistance);
        }

        private void UpdateTransform()
        {
            var rotation = Quaternion.Euler(_pitch, _yaw, 0f);
            var offset = rotation * new Vector3(0f, 0f, -_distance);
            transform.position = _pivot + offset;
            transform.rotation = rotation;
        }
    }
}
