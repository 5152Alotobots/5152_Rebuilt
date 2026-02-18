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
package frc.alotobots.rebuilt.subsystems.intake.extendo.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtendoIO {
  enum PIDSlots {
    VELOCITY,
    MOTION_MAGIC_POSITION,
  }

  /** Data structure for inputs from intake extendo hardware. */
  @AutoLog
  public static class IntakeExtendoIOInputs {
    public PIDSlots intakeExtendoMotorPIDSlot = PIDSlots.VELOCITY;

    public boolean intakeExtendoMotorConnected = false;

    public Distance intakeExtendoDistance = Meters.zero();
    public Angle intakeExtendoMotorAngle = Rotations.zero();
    public AngularVelocity intakeExtendoMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration intakeExtendoMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage intakeExtendoMotorVolts = Volts.zero();
    public Current intakeExtendoMotorCurrent = Amps.zero();
  }

  /**
   * Updates the intake roller input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(IntakeExtendoIOInputs inputs) {}

  /**
   * Sets the target position for the extendo using closed-loop motion-magic control.
   *
   * @param position The desired position for the extendo
   */
  public default void setIntakeExtendoPosition(Distance position, PIDSlots pidSlot) {}

  /**
   * Sets the target position for the extendo using closed-loop motion-magic control.
   *
   * @param position The desired position for the extendo
   */
  public default void setIntakeExtendoPosition(Distance position) {}

  /**
   * Sets the intake extendo to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (optional)
   */
  default void setIntakeExtendoVelocity(LinearVelocity velocity, PIDSlots pidSlot) {}

  /**
   * Sets the intake extendo to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   */
  default void setIntakeExtendoVelocity(LinearVelocity velocity) {}

  /**
   * Runs the intake extendo using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setIntakeExtendoOpenLoop(double percentOutput) {}

  /** Stops all intake extendo motor movement. */
  default void stop() {}
}
