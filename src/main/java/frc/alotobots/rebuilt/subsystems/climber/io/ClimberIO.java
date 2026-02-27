package frc.alotobots.rebuilt.subsystems.climber.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.ControlModeValue;

public interface ClimberIO {
  enum PIDSlots {
    VELOCITY,
    POSITION,
    MOTION_MAGIC_POSITION,
  }

  /** Data structure for inputs from intake extendo hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    public PIDSlots climberMotorPIDSlot = PIDSlots.VELOCITY;
    public ControlModeValue climberMotorControlMode = ControlModeValue.DisabledOutput;
    public boolean climberMotorConnected = false;
    public Distance climberDistance = Meters.zero();
    public Angle climberMotorAngle = Rotations.zero();
    public AngularVelocity climberMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration climberMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage climberMotorVolts = Volts.zero();
    public Current climberMotorCurrent = Amps.zero();
  }

  /**
   * Updates the climber input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(ClimberIOInputs inputs) {}
  /**
   * Sets the target position for the climber using closed-loop motion-magic control.
   *
   * @param position The desired position for the climber
   */
  public default void setClimberPosition(Distance position, PIDSlots pidSlot) {}

  /**
   * Sets the target position for the climber using closed-loop motion-magic control.
   *
   * @param position The desired position for the climber
   */
  public default void setClimberPosition(Distance position) {}
  /**
   * Sets the climber to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (optional)
   */
  default void setClimberVelocity(LinearVelocity velocity, PIDSlots pidSlot) {}

  /**
   * Sets the climber to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   */
  default void setClimberVelocity(LinearVelocity velocity) {}

  /**
   * Runs the climber using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setClimberOpenLoop(double percentOutput) {}
  /** Stops all climber motor movement. */
  default void stop() {}
}
