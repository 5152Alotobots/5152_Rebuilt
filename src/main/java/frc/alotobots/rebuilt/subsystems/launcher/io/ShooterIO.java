package frc.alotobots.rebuilt.subsystems.launcher.io;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;



public interface ShooterIO {
    enum PIDSlots {
        DEFAULT_VELOCITY,
    }

    /** Data structure for inputs from shooter hardware. */
    @AutoLog
    public static class ShooterIOInputs {
        /** Current PID slot being used (0 for velocity, 1 for position) */
        public int motorLeftPIDSlot = 0;
        public int motorRightPIDSlot = 0;

        public boolean motorLeftConnected = false;
        public boolean motorRightConnected = false;

        public AngularVelocity motorLeftVelocity = RotationsPerSecond.zero();
        public AngularVelocity motorRightVelocity = RotationsPerSecond.zero();

        public AngularAcceleration motorLeftAcceleration = RotationsPerSecondPerSecond.zero();
        public AngularAcceleration motorRightAcceleration = RotationsPerSecondPerSecond.zero();

        public Voltage motorLeftAppliedVolts = Volts.zero();
        public Voltage motorRightAppliedVolts = Volts.zero();

        public Current motorLeftCurrent = Amps.zero();
        public Current motorRightCurrent = Amps.zero();
    }

    /**
     * Updates the shooter input values from hardware.
     *
     * @param inputs The input object to update with the latest hardware state
     */
    default void updateInputs(ShooterIO.ShooterIOInputs inputs) {}

    /**
     * Sets the shooter to run at a target velocity using closed-loop control.
     *
     * @param velocity The target velocity to move at
     * @param pidSlot The PID slot to use (optional)
     */
    default void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {}

    /**
     * Sets the shooter to run at a target velocity using closed-loop control.
     *
     * @param velocity The target velocity to move at
     */
    default void setShooterVelocity(AngularVelocity velocity) {}

    /**
     * Runs the shooter using direct percentage output (open-loop control).
     *
     * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
     */
    default void setShooterOpenLoop(double percentOutput) {}

    /** Stops all shooter motor movement. */
    default void stop() {}
}
