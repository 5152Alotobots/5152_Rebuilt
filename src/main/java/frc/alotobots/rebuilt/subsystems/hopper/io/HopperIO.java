package frc.alotobots.rebuilt.subsystems.hopper.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

interface HopperIO {
    enum PIDSlots {
        DEFAULT_VELOCITY,
    }

  /** Data structure for inputs from shooter hardware. */
  @AutoLog
  public static class HopperIOInputs {
    public PIDSlots motorKickerPIDSlot = PIDSlots.DEFAULT_VELOCITY;
    public boolean motorKickerConnected = false;
    public AngularVelocity motorKickerVelocity = RotationsPerSecond.zero();
    public AngularAcceleration motorKickerAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage motorKickerAppliedVolts = Volts.zero();
    public Current motorKickerCurrent = Amps.zero();
  }

  /**
   * Updates the shooter input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(HopperIO.HopperIOInputs inputs) {}

  /**
   * Sets the kicker to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (optional)
   */
  default void setKickerVelocity(AngularVelocity velocity, PIDSlots pidSlot) {}

  /**
   * Sets the kicker to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   */
  default void setKickerVelocity(AngularVelocity velocity) {}

  /**
   * Runs the kicker using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setKickerOpenLoop(double percentOutput) {}

  /** Stops all kicker motor movement. */
  default void stop() {}
}
