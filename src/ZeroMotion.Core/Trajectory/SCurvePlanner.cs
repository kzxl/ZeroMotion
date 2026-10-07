using System;

namespace ZeroMotion.Trajectory
{
    /// <summary>
    /// 7-Phase Jerk-Limited S-Curve Motion Profile Generator.
    /// Eliminates mechanical resonance, vibration, and gear backlash by constraining jerk (da/dt &lt;= J_max).
    /// </summary>
    public class SCurvePlanner
    {
        public double StartPosition { get; set; }
        public double TargetPosition { get; set; }
        public double MaxVelocity { get; set; }     // v_max > 0
        public double MaxAcceleration { get; set; } // a_max > 0
        public double MaxJerk { get; set; }         // j_max > 0

        // Calculated phase durations
        public double Tj1 { get; private set; } // Jerk ramp-up time (a: 0 -> a_max)
        public double Ta { get; private set; }  // Constant acceleration time
        public double Tj2 { get; private set; } // Jerk ramp-down time (a: a_max -> 0)
        public double Tv { get; private set; }  // Constant velocity cruise time
        public double Td { get; private set; }  // Constant deceleration time
        public double TotalDuration { get; private set; }

        public SCurvePlanner(double startPos, double targetPos, double maxVel, double maxAcc, double maxJerk)
        {
            if (maxVel <= 0) throw new ArgumentOutOfRangeException(nameof(maxVel), "Max velocity must be positive.");
            if (maxAcc <= 0) throw new ArgumentOutOfRangeException(nameof(maxAcc), "Max acceleration must be positive.");
            if (maxJerk <= 0) throw new ArgumentOutOfRangeException(nameof(maxJerk), "Max jerk must be positive.");

            StartPosition = startPos;
            TargetPosition = targetPos;
            MaxVelocity = maxVel;
            MaxAcceleration = maxAcc;
            MaxJerk = maxJerk;

            Plan();
        }

        private void Plan()
        {
            double distance = System.Math.Abs(TargetPosition - StartPosition);
            if (distance < 1e-9)
            {
                Tj1 = Ta = Tj2 = Tv = Td = TotalDuration = 0.0;
                return;
            }

            double vMax = MaxVelocity;
            double aMax = MaxAcceleration;
            double jMax = MaxJerk;

            // Check if aMax is reached
            if (vMax * jMax < aMax * aMax)
            {
                // Acceleration limit cannot be reached; reduce aMax
                aMax = System.Math.Sqrt(vMax * jMax);
            }

            Tj1 = aMax / jMax;
            Ta = (vMax / aMax) - Tj1;
            Tj2 = Tj1;

            // Distance covered during acceleration phase
            double sAccel = vMax * (Ta + Tj1);

            if (distance >= 2.0 * sAccel)
            {
                // Full cruise velocity is reached
                Tv = (distance - 2.0 * sAccel) / vMax;
                Td = Ta;
            }
            else
            {
                // Cruise phase does not exist (Tv = 0)
                Tv = 0.0;
                // Recompute actual peak velocity reached
                double gamma = distance / (2.0 * Tj1);
                double vAct = (jMax * Tj1 * Tj1 * 0.5) + (jMax * Tj1 * System.Math.Sqrt(Tj1 * Tj1 * 0.25 + distance / jMax));
                vAct = System.Math.Min(vMax, System.Math.Sqrt(distance * aMax));

                Ta = (vAct / aMax) - Tj1;
                if (Ta < 0.0)
                {
                    Ta = 0.0;
                    Tj1 = System.Math.Pow(distance * 0.5 / jMax, 1.0 / 3.0);
                    Tj2 = Tj1;
                }
                Td = Ta;
            }

            TotalDuration = 4.0 * Tj1 + 2.0 * Ta + Tv;
        }

        /// <summary>
        /// Evaluates current position, velocity, and acceleration at time t (seconds).
        /// </summary>
        public (double Position, double Velocity, double Acceleration) Evaluate(double t)
        {
            if (TotalDuration < 1e-9) return (TargetPosition, 0.0, 0.0);

            double dir = TargetPosition >= StartPosition ? 1.0 : -1.0;
            double s0 = StartPosition;
            double j = MaxJerk;
            double a = MaxAcceleration;
            double v = MaxVelocity;

            t = System.Math.Max(0.0, System.Math.Min(TotalDuration, t));

            double t1 = Tj1;
            double t2 = t1 + Ta;
            double t3 = t2 + Tj1;
            double t4 = t3 + Tv;
            double t5 = t4 + Tj1;
            double t6 = t5 + Td;
            double t7 = TotalDuration;

            double pos, vel, acc;

            if (t <= t1)
            {
                // Phase 1: Jerk ramp-up
                acc = j * t;
                vel = 0.5 * j * t * t;
                pos = s0 + dir * (j * t * t * t / 6.0);
            }
            else if (t <= t2)
            {
                // Phase 2: Constant acceleration
                double dt = t - t1;
                double v1 = 0.5 * j * t1 * t1;
                double s1 = j * t1 * t1 * t1 / 6.0;

                acc = a;
                vel = v1 + a * dt;
                pos = s0 + dir * (s1 + v1 * dt + 0.5 * a * dt * dt);
            }
            else if (t <= t3)
            {
                // Phase 3: Jerk ramp-down to cruise
                double dt = t - t2;
                double v1 = 0.5 * j * t1 * t1;
                double v2 = v1 + a * Ta;
                double s1 = j * t1 * t1 * t1 / 6.0;
                double s2 = s1 + v1 * Ta + 0.5 * a * Ta * Ta;

                acc = a - j * dt;
                vel = v2 + a * dt - 0.5 * j * dt * dt;
                pos = s0 + dir * (s2 + v2 * dt + 0.5 * a * dt * dt - (j * dt * dt * dt / 6.0));
            }
            else if (t <= t4)
            {
                // Phase 4: Cruise velocity
                double dt = t - t3;
                double v3 = v;
                double s3 = 0.5 * (TargetPosition - StartPosition) - 0.5 * v * Tv;

                acc = 0.0;
                vel = v3;
                pos = s0 + dir * (System.Math.Abs(s3) + v3 * dt);
            }
            else if (t <= t5)
            {
                // Phase 5: Deceleration jerk ramp-up
                double dt = t - t4;
                acc = -j * dt;
                vel = v - 0.5 * j * dt * dt;
                pos = TargetPosition - dir * (v * (t7 - t) - (j * System.Math.Pow(t7 - t, 3) / 6.0));
            }
            else if (t <= t6)
            {
                // Phase 6: Constant deceleration
                double dt = t - t5;
                acc = -a;
                vel = v - 0.5 * j * Tj1 * Tj1 - a * dt;
                pos = TargetPosition - dir * (0.5 * a * (t7 - t) * (t7 - t));
            }
            else
            {
                // Phase 7: Deceleration jerk ramp-down to zero
                double dt = t7 - t;
                acc = -j * dt;
                vel = 0.5 * j * dt * dt;
                pos = TargetPosition - dir * (j * dt * dt * dt / 6.0);
            }

            return (pos, vel * dir, acc * dir);
        }
    }
}
