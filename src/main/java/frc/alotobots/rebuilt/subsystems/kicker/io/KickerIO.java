/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.kicker.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  enum PIDSlots {
    DEFAULT_VELOCITY,
  }

  /** Data structure for inputs from kicker hardware. */
  @AutoLog
  public static class KickerIOInputs {
    public PIDSlots kickerMotorPIDSlot = PIDSlots.DEFAULT_VELOCITY;
    public boolean kickerMotorConnected = false;
    public AngularVelocity kickerMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration kickerMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage kickerMotorVolts = Volts.zero();
    public Current kickerMotorCurrent = Amps.zero();
  }

  /**
   * Updates the shooter input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(KickerIO.KickerIOInputs inputs) {}

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
