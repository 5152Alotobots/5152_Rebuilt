/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2026 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.climber.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  enum PIDSlots {
    VELOCITY,
    POSITION,
    MOTION_MAGIC_POSITION,
  }

  /** Data structure for inputs from intake extendo hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    public PIDSlots climberPIDSlot = PIDSlots.VELOCITY;
    public ControlModeValue climberControlMode = ControlModeValue.DisabledOutput;
    public boolean climberConnected = false;
    public Distance climberDistance = Meters.zero();
    public Angle climberAngle = Rotations.zero();
    public AngularVelocity climberVelocity = RotationsPerSecond.zero();
    public AngularAcceleration climberAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage climberVolts = Volts.zero();
    public Current climberCurrent = Amps.zero();
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
