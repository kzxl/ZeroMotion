using System;
using Xunit;
using ZeroMotion.Kinematics;
using ZeroMotion.Math;

namespace ZeroMotion.Tests
{
    public class KinematicsTests
    {
        [Fact]
        public void Standard6AxisRobot_ComputesForwardKinematicsAtHomePosition()
        {
            var robot = RobotKinematics.CreateStandard6AxisRobot(d1: 400f, a1: 150f, a2: 450f, d4: 400f, d6: 100f);
            var fk = robot.ComputeForwardKinematics(new float[] { 0f, 0f, 0f, 0f, 0f, 0f });

            Assert.Equal(7, fk.JointTransforms.Count); // Base + 6 links
            Assert.Equal(7, fk.JointPositions.Count);

            var ee = fk.EndEffectorPosition;
            Assert.True(ee.Length() > 500f, $"End-effector reach was {ee.Length()} mm");
        }

        [Fact]
        public void ScaraRobot_AnalyticalIkMatchesFkExactly()
        {
            float l1 = 300f;
            float l2 = 250f;
            float baseH = 200f;
            var target = new Vec3(350f, 150f, 100f);

            var sol = AnalyticalInverseKinematics.SolveScara(l1, l2, baseH, target, targetRollDeg: 0f, elbowRight: true);
            Assert.True(sol.IsReachable);

            var scara = RobotKinematics.CreateStandardScaraRobot(l1, l2, baseH);
            var fk = scara.ComputeForwardKinematics(new float[] { sol.Theta1Deg, sol.Theta2Deg, sol.ZPrismatic, sol.Theta4Deg });

            float dist = Vec3.Distance(fk.EndEffectorPosition, target);
            Assert.InRange(dist, 0f, 0.5f);
        }

        [Fact]
        public void ScaraRobot_CcdNumericalIkConvergesWithinTolerance()
        {
            var robot = RobotKinematics.CreateStandardScaraRobot(l1: 300f, l2: 250f, baseHeight: 200f);
            var target = new Vec3(350f, 150f, 100f);

            var ikResult = NumericalInverseKinematics.SolveCcd(
                robot,
                target,
                seedJoints: new float[] { 0f, 0f, 0f, 0f },
                maxIterations: 100,
                toleranceMm: 1.0f,
                damping: 0.8f
            );

            Assert.True(ikResult.Success, $"SCARA CCD-IK should converge. Error: {ikResult.FinalErrorMm:F2} mm");
            Assert.InRange(ikResult.FinalErrorMm, 0f, 1.0f);

            var fk = robot.ComputeForwardKinematics(ikResult.JointAngles);
            Assert.InRange(Vec3.Distance(fk.EndEffectorPosition, target), 0f, 1.0f);
        }

        [Fact]
        public void ComputeJacobian_ComputesProperDimensionsAndNonSingularValues()
        {
            var robot = RobotKinematics.CreateStandard6AxisRobot();
            var joints = new float[] { 10f, -20f, 30f, 0f, 45f, 0f };

            var J = robot.ComputeJacobian(joints);

            Assert.Equal(6, J.GetLength(0)); // 6 rows (v_x, v_y, v_z, w_x, w_y, w_z)
            Assert.Equal(6, J.GetLength(1)); // 6 columns (joints)

            bool hasNonZero = false;
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    if (System.Math.Abs(J[r, c]) > 1e-3f) hasNonZero = true;
                }
            }

            Assert.True(hasNonZero, "Jacobian must contain non-zero motion derivatives.");
        }

        [Fact]
        public void ComputeManipulability_DetectsSingularityAndNormalState()
        {
            var robot = RobotKinematics.CreateStandard6AxisRobot();
            // Posture away from wrist and shoulder singularities
            var nonSingularJoints = new float[] { 15f, 30f, -45f, 25f, 45f, 20f };
            float wNonSingular = robot.ComputeManipulability(nonSingularJoints);
            Assert.True(wNonSingular > 0f, $"Manipulability in non-singular posture should be positive, was {wNonSingular}");

            // Posture with wrist singularity (Joint 5 at 0 degrees aligns Joint 4 and Joint 6)
            var singularJoints = new float[] { 0f, 0f, 0f, 0f, 0f, 0f };
            float wSingular = robot.ComputeManipulability(singularJoints);
            Assert.Equal(0f, wSingular, 3); // Must detect singularity

            // DLS solver test: remains stable and bounded even near singularity
            var deltaLinear = new Vec3(1.0f, 0f, 0f); // 1mm in X
            var deltaAngular = Vec3.Zero;
            var deltaQ = robot.SolveDampedLeastSquaresIK(nonSingularJoints, deltaLinear, deltaAngular, dampingLambda: 0.1f);

            Assert.Equal(6, deltaQ.Length);
            for (int i = 0; i < 6; i++)
            {
                Assert.False(float.IsNaN(deltaQ[i]), $"Joint delta {i} was NaN");
                Assert.False(float.IsInfinity(deltaQ[i]), $"Joint delta {i} was Infinity");
            }
        }
    }
}
