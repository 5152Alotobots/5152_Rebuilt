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
package frc.alotobots.rebuilt.subsystems.roller.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  enum PIDSlots {
    DEFAULT_VELOCITY,
  }

  /** Data structure for inputs from belt hardware. */
  @AutoLog
  public static class RollerIOInputs {
    public PIDSlots rollerMotorPIDSlot = PIDSlots.DEFAULT_VELOCITY;
    public boolean rollerMotorConnected = false;
    public AngularVelocity rollerMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration rollerMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage rollerMotorVolts = Volts.zero();
    public Current rollerMotorCurrent = Amps.zero();
  }

  /**
   * Updates the roller input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(RollerIO.RollerIOInputs inputs) {}

  /**
   * Sets the roller to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (optional)
   */
  default void setRollerVelocity(AngularVelocity velocity, PIDSlots pidSlot) {}

  /**
   * Sets the belt to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   */
  default void setRollerVelocity(AngularVelocity velocity) {}

  /**
   * Runs the kicker using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setRollerOpenLoop(double percentOutput) {}

  /** Stops all belt motor movement. */
  default void stop() {}
}
