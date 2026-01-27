package frc.alotobots.rebuilt.subsystems.turret.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
        /** Data structure for inputs from turret hardware. */
        @AutoLog
        public static class TurretIOInputs {
            /** Current PID slot being used (0 for velocity, 1 for position) */
            public int pidSlot = 0;

            /** Whether the motor controller is connected */
            public boolean motorConnected = false;

            /** Whether the top limit switch/soft limit is triggered */
            public boolean ccwLimit = false;

            /** Whether the bottom limit switch/soft limit is triggered */
            public boolean cwLimit = false;

            /** Current angle of the turret mechanism */
            public Angle mechanismAngle = Rotations.zero();

            /** Current angular velocity of the turret */
            public AngularVelocity rotationVelocity = RotationsPerSecond.zero();

            public AngularAcceleration rotationAcceleration = RotationsPerSecondPerSecond.zero();

            /** Current voltage being applied to the motor */
            public Voltage motorAppliedVolts = Volts.zero();

            /** Current being drawn by the motor */
            public Current motorCurrent = Amps.zero();
        }

        /**
         * Updates the turret input values from hardware.
         *
         * @param inputs The input object to update with the latest hardware state
         */
        default void updateInputs(TurretIOInputs inputs) {}

        /**
         * Sets the turret to run to a target position using closed-loop control.
         *
         * @param position The target angle to move to
         * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
         */
        default void setTurretPosition(Angle position, int pidSlot) {}

        /**
         * Sets the turret to run at a target velocity using closed-loop control.
         *
         * @param velocity The target velocity to move at
         * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
         */
        default void setTurretVelocity(AngularVelocity velocity, int pidSlot) {}

        /**
         * Runs the turret using direct percentage output (open-loop control).
         *
         * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
         */
        default void setTurretOpenLoop(double percentOutput) {}

        /** Stops all turret motor movement. */
        default void stop() {}
}
