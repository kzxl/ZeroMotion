using System;
using ZeroMotion.Math;

namespace ZeroMotion.Kinematics
{
    /// <summary>
    /// Analytical closed-form Inverse Kinematics solver.
    /// Provides deterministic, sub-microsecond, exact solutions without numerical iterations for SCARA and spherical wrist manipulators.
    /// </summary>
    public static class AnalyticalInverseKinematics
    {
        public struct ScaraSolution
        {
            public bool IsReachable;
            public float Theta1Deg; // Shoulder
            public float Theta2Deg; // Elbow
            public float ZPrismatic; // Prismatic Z
            public float Theta4Deg;  // Tool Yaw
        }

        public struct WristSolution
        {
            public float Theta4Deg;
            public float Theta5Deg;
            public float Theta6Deg;
        }

        /// <summary>
        /// Solves exact closed-form inverse kinematics for a 4-Axis SCARA robot.
        /// </summary>
        /// <param name="l1">Length of link 1 (shoulder to elbow).</param>
        /// <param name="l2">Length of link 2 (elbow to wrist).</param>
        /// <param name="baseHeight">Base height along Z axis.</param>
        /// <param name="target">Target 3D position.</param>
        /// <param name="targetRollDeg">Desired tool roll angle in degrees.</param>
        /// <param name="elbowRight">True for elbow-right configuration, false for elbow-left.</param>
        public static ScaraSolution SolveScara(
            float l1, float l2, float baseHeight, Vec3 target, float targetRollDeg = 0f, bool elbowRight = true)
        {
            float x = target.X;
            float y = target.Y;
            float rSq = x * x + y * y;
            float r = (float)System.Math.Sqrt(rSq);

            // Reachability check
            if (r > (l1 + l2) || r < System.Math.Abs(l1 - l2))
            {
                return new ScaraSolution { IsReachable = false };
            }

            // Cosine rule for theta2
            float cosTheta2 = (rSq - l1 * l1 - l2 * l2) / (2f * l1 * l2);
            cosTheta2 = System.Math.Max(-1.0f, System.Math.Min(1.0f, cosTheta2));
            float sinTheta2 = (float)System.Math.Sqrt(System.Math.Max(0f, 1f - cosTheta2 * cosTheta2));
            if (!elbowRight) sinTheta2 = -sinTheta2;

            float theta2Rad = (float)System.Math.Atan2(sinTheta2, cosTheta2);

            // Theta1
            float beta = (float)System.Math.Atan2(y, x);
            float gamma = (float)System.Math.Atan2(l2 * sinTheta2, l1 + l2 * cosTheta2);
            float theta1Rad = beta - gamma;

            float theta1Deg = theta1Rad * 180f / (float)System.Math.PI;
            float theta2Deg = theta2Rad * 180f / (float)System.Math.PI;
            float zPrismatic = target.Z - baseHeight;
            float theta4Deg = targetRollDeg - (theta1Deg + theta2Deg);

            return new ScaraSolution
            {
                IsReachable = true,
                Theta1Deg = theta1Deg,
                Theta2Deg = theta2Deg,
                ZPrismatic = zPrismatic,
                Theta4Deg = theta4Deg
            };
        }

        /// <summary>
        /// Solves the orientation inverse kinematics for a 3-axis spherical wrist (Joints 4, 5, 6) given the wrist rotation matrix R_3_6.
        /// </summary>
        public static WristSolution SolveSphericalWrist(Mat4 r36)
        {
            float r13 = r36.M13;
            float r23 = r36.M23;
            float r33 = r36.M33;
            float r31 = r36.M31;
            float r32 = r36.M32;

            float theta5Rad = (float)System.Math.Atan2(System.Math.Sqrt(r13 * r13 + r23 * r23), r33);
            float theta4Rad, theta6Rad;

            if (System.Math.Abs(theta5Rad) > 1e-4f)
            {
                theta4Rad = (float)System.Math.Atan2(r23, r13);
                theta6Rad = (float)System.Math.Atan2(r32, -r31);
            }
            else
            {
                theta4Rad = 0f;
                theta6Rad = (float)System.Math.Atan2(-r36.M21, r36.M11);
            }

            return new WristSolution
            {
                Theta4Deg = theta4Rad * 180f / (float)System.Math.PI,
                Theta5Deg = theta5Rad * 180f / (float)System.Math.PI,
                Theta6Deg = theta6Rad * 180f / (float)System.Math.PI
            };
        }
    }
}
