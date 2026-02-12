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
package frc.alotobots.rebuilt.subsystems.belt.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {
  enum PIDSlots {
    DEFAULT_VELOCITY,
  }

  /** Data structure for inputs from belt hardware. */
  @AutoLog
  public static class BeltIOInputs {
    public PIDSlots beltMotorPIDSlot = PIDSlots.DEFAULT_VELOCITY;
    public boolean beltMotorConnected = false;
    public AngularVelocity beltMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration beltMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage beltMotorVolts = Volts.zero();
    public Current beltMotorCurrent = Amps.zero();
  }

  /**
   * Updates the shooter input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(BeltIO.BeltIOInputs inputs) {}

  /**
   * Sets the kicker to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (optional)
   */
  default void setBeltVelocity(AngularVelocity velocity, PIDSlots pidSlot) {}

  /**
   * Sets the belt to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   */
  default void setBeltVelocity(AngularVelocity velocity) {}

  /**
   * Runs the kicker using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setBeltOpenLoop(double percentOutput) {}

  /** Stops all belt motor movement. */
  default void stop() {}
}
