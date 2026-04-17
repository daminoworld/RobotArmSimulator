using System.Collections.Generic;
using UnityEngine;

namespace RobotArmSimulator
{
    public static class CoordinateTransformUtility
    {
        public static Matrix4x4 BuildSourceToUnityBasis(AxisConventionData sourceAxis, AxisConventionData unityAssumption)
        {
            var sourceXInUnity = DirectionSemanticToUnityVector(sourceAxis != null ? sourceAxis.X : "forward", unityAssumption);
            var sourceYInUnity = DirectionSemanticToUnityVector(sourceAxis != null ? sourceAxis.Y : "left", unityAssumption);
            var sourceZInUnity = DirectionSemanticToUnityVector(sourceAxis != null ? sourceAxis.Z : "up", unityAssumption);

            if (sourceXInUnity == Vector3.zero || sourceYInUnity == Vector3.zero || sourceZInUnity == Vector3.zero)
            {
                sourceXInUnity = Vector3.forward;
                sourceYInUnity = Vector3.left;
                sourceZInUnity = Vector3.up;
            }

            var basis = Matrix4x4.identity;
            basis.SetColumn(0, new Vector4(sourceXInUnity.x, sourceXInUnity.y, sourceXInUnity.z, 0f));
            basis.SetColumn(1, new Vector4(sourceYInUnity.x, sourceYInUnity.y, sourceYInUnity.z, 0f));
            basis.SetColumn(2, new Vector4(sourceZInUnity.x, sourceZInUnity.y, sourceZInUnity.z, 0f));
            basis.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
            return basis;
        }

        public static Matrix4x4 BuildRotationFromEulerZyx(Vector3 eulerZyxDeg)
        {
            var rz = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerZyxDeg.x, Vector3.forward));
            var ry = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerZyxDeg.y, Vector3.up));
            var rx = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerZyxDeg.z, Vector3.right));
            return rz * ry * rx;
        }

        public static Matrix4x4 BuildRotationFromEulerXyz(Vector3 eulerXyzDeg)
        {
            var rx = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerXyzDeg.x, Vector3.right));
            var ry = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerXyzDeg.y, Vector3.up));
            var rz = Matrix4x4.Rotate(Quaternion.AngleAxis(eulerXyzDeg.z, Vector3.forward));
            return rx * ry * rz;
        }

        public static Matrix4x4 BuildTransform(Vector3 translation, Matrix4x4 rotation)
        {
            var result = rotation;
            result.m03 = translation.x;
            result.m13 = translation.y;
            result.m23 = translation.z;
            result.m30 = 0f;
            result.m31 = 0f;
            result.m32 = 0f;
            result.m33 = 1f;
            return result;
        }

        public static Matrix4x4 ConvertSourceMatrixToUnity(Matrix4x4 sourceMatrix, Matrix4x4 sourceToUnityBasis)
        {
            var unityToSourceBasis = sourceToUnityBasis.inverse;
            return sourceToUnityBasis * sourceMatrix * unityToSourceBasis;
        }

        public static Vector3 ExtractPosition(Matrix4x4 matrix)
        {
            return new Vector3(matrix.m03, matrix.m13, matrix.m23);
        }

        public static Quaternion ExtractRotation(Matrix4x4 matrix)
        {
            var forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
            var up = new Vector3(matrix.m01, matrix.m11, matrix.m21);
            if (forward.sqrMagnitude < 0.000001f || up.sqrMagnitude < 0.000001f)
            {
                return Quaternion.identity;
            }

            return Quaternion.LookRotation(forward.normalized, up.normalized);
        }

        private static Vector3 DirectionSemanticToUnityVector(string semantic, AxisConventionData unityAssumption)
        {
            var normalized = NormalizeSemantic(semantic);
            if (string.IsNullOrWhiteSpace(normalized))
            {
                return Vector3.zero;
            }

            var positiveByAxis = new Dictionary<string, string>
            {
                { "x", NormalizeSemantic(unityAssumption != null ? unityAssumption.X : "right") },
                { "y", NormalizeSemantic(unityAssumption != null ? unityAssumption.Y : "up") },
                { "z", NormalizeSemantic(unityAssumption != null ? unityAssumption.Z : "forward") }
            };

            if (normalized == positiveByAxis["x"])
            {
                return Vector3.right;
            }

            if (normalized == OppositeSemantic(positiveByAxis["x"]))
            {
                return Vector3.left;
            }

            if (normalized == positiveByAxis["y"])
            {
                return Vector3.up;
            }

            if (normalized == OppositeSemantic(positiveByAxis["y"]))
            {
                return Vector3.down;
            }

            if (normalized == positiveByAxis["z"])
            {
                return Vector3.forward;
            }

            if (normalized == OppositeSemantic(positiveByAxis["z"]))
            {
                return Vector3.back;
            }

            return Vector3.zero;
        }

        private static string NormalizeSemantic(string raw)
        {
            return string.IsNullOrWhiteSpace(raw) ? string.Empty : raw.Trim().ToLowerInvariant();
        }

        private static string OppositeSemantic(string semantic)
        {
            switch (NormalizeSemantic(semantic))
            {
                case "right":
                    return "left";
                case "left":
                    return "right";
                case "up":
                    return "down";
                case "down":
                    return "up";
                case "forward":
                    return "backward";
                case "backward":
                    return "forward";
                default:
                    return string.Empty;
            }
        }
    }
}
