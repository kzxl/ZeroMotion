using System;
using System.Collections.Generic;
using ZeroMotion.Math;

namespace ZeroMotion.Kinematics
{
    /// <summary>
    /// Multi-axis industrial robot kinematics engine.
    /// Computes forward kinematics, geometric Jacobians, and frame transforms for N-DOF manipulators.
    /// </summary>
    public class RobotKinematics
    {
        public List<DhLink> Links { get; } = new List<DhLink>();
        public Mat4 BaseTransform { get; set; } = Mat4.Identity;
        public Mat4 ToolTransform { get; set; } = Mat4.Identity; // TCP (Tool Center Point)

        public RobotKinematics()
        {
        }

        public RobotKinematics AddLink(DhLink link)
        {
            Links.Add(link);
            return this;
        }

        public KinematicsResult ComputeForwardKinematics(IReadOnlyList<float> jointValues)
        {
            if (jointValues == null) throw new ArgumentNullException(nameof(jointValues));

            var result = new KinematicsResult();
            var current = BaseTransform;

            result.JointTransforms.Add(current);
            result.JointPositions.Add(current.Translation);

            for (int i = 0; i < Links.Count; i++)
            {
                float val = (i < jointValues.Count) ? jointValues[i] : 0f;
                Mat4 localT = Links[i].ComputeTransform(val);
                current = localT * current;

                result.JointTransforms.Add(current);
                result.JointPositions.Add(current.Translation);
            }

            result.EndEffectorTransform = ToolTransform * current;
            return result;
        }

        /// <summary>
        /// Computes the 6xN Geometric Jacobian matrix J(q) relating joint velocities to end-effector linear & angular velocity.
        /// [v; omega] = J(q) * q_dot
        /// Returns float[6, N] array where rows 0..2 are linear velocity and rows 3..5 are angular velocity.
        /// </summary>
        public float[,] ComputeJacobian(IReadOnlyList<float> jointValues)
        {
            int n = Links.Count;
            var jacobian = new float[6, n];
            var fk = ComputeForwardKinematics(jointValues);
            var pe = fk.EndEffectorPosition;

            for (int i = 0; i < n; i++)
            {
                var t = fk.JointTransforms[i];
                var pi = t.Translation;
                var zi = new Vec3(t.M31, t.M32, t.M33).Normalize();
                if (zi.LengthSquared() < 1e-4f) zi = Vec3.UnitZ;

                if (Links[i].Type == JointType.Revolute)
                {
                    var jv = Vec3.Cross(zi, pe - pi);
                    jacobian[0, i] = jv.X;
                    jacobian[1, i] = jv.Y;
                    jacobian[2, i] = jv.Z;

                    jacobian[3, i] = zi.X;
                    jacobian[4, i] = zi.Y;
                    jacobian[5, i] = zi.Z;
                }
                else
                {
                    jacobian[0, i] = zi.X;
                    jacobian[1, i] = zi.Y;
                    jacobian[2, i] = zi.Z;

                    jacobian[3, i] = 0f;
                    jacobian[4, i] = 0f;
                    jacobian[5, i] = 0f;
                }
            }

            return jacobian;
        }

        /// <summary>
        /// Creates a standard 6-axis articulated industrial robot manipulator (e.g. FANUC / ABB / KUKA kinematics).
        /// </summary>
        public static RobotKinematics CreateStandard6AxisRobot(float d1 = 400f, float a1 = 150f, float a2 = 450f, float d4 = 400f, float d6 = 100f)
        {
            var robot = new RobotKinematics();
            robot.AddLink(new DhLink(0f, d1, a1, 90f, "Joint1_BaseYaw").WithLimits(-170f, 170f));
            robot.AddLink(new DhLink(0f, 0f, a2, 0f, "Joint2_ShoulderPitch").WithLimits(-100f, 135f));
            robot.AddLink(new DhLink(0f, 0f, 0f, 90f, "Joint3_ElbowPitch").WithLimits(-200f, 65f));
            robot.AddLink(new DhLink(0f, d4, 0f, -90f, "Joint4_WristRoll").WithLimits(-190f, 190f));
            robot.AddLink(new DhLink(0f, 0f, 0f, 90f, "Joint5_WristPitch").WithLimits(-125f, 125f));
            robot.AddLink(new DhLink(0f, d6, 0f, 0f, "Joint6_FlangeRoll").WithLimits(-360f, 360f));
            return robot;
        }

        /// <summary>
        /// Creates a 4-DOF SCARA (Selective Compliance Articulated Robot Arm) manipulator.
        /// </summary>
        public static RobotKinematics CreateStandardScaraRobot(float l1 = 300f, float l2 = 250f, float baseHeight = 200f)
        {
            var robot = new RobotKinematics();
            robot.AddLink(new DhLink(0f, baseHeight, l1, 0f, "Joint1_Arm1Yaw").WithLimits(-135f, 135f));
            robot.AddLink(new DhLink(0f, 0f, l2, 0f, "Joint2_Arm2Yaw").WithLimits(-150f, 150f));
            var prismaticLink = new DhLink(0f, 0f, 0f, 0f, "Joint3_ZPrismatic")
            {
                Type = JointType.Prismatic,
                MinLimit = -200f,
                MaxLimit = 0f
            };
            robot.AddLink(prismaticLink);
            robot.AddLink(new DhLink(0f, 0f, 0f, 0f, "Joint4_ToolYaw").WithLimits(-360f, 360f));
            return robot;
        }

        /// <summary>
        /// Creates a 3-Axis Cartesian (Gantry) Robot.
        /// </summary>
        public static RobotKinematics CreateCartesianRobot(float xMax = 1000f, float yMax = 1000f, float zMax = 500f)
        {
            var robot = new RobotKinematics();
            var jX = new DhLink(0f, 0f, 0f, -90f, "Axis_X") { Type = JointType.Prismatic, MinLimit = 0f, MaxLimit = xMax };
            var jY = new DhLink(-90f, 0f, 0f, -90f, "Axis_Y") { Type = JointType.Prismatic, MinLimit = 0f, MaxLimit = yMax };
            var jZ = new DhLink(0f, 0f, 0f, 0f, "Axis_Z") { Type = JointType.Prismatic, MinLimit = 0f, MaxLimit = zMax };
            robot.AddLink(jX).AddLink(jY).AddLink(jZ);
            return robot;
        }
    }
}
