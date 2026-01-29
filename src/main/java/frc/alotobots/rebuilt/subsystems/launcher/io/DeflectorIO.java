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
package frc.alotobots.rebuilt.subsystems.launcher.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface DeflectorIO {
  enum PIDSlots {
    DEFAULT_POSITION,
  }

  @AutoLog
  public static class DeflectorIOInputs {
    public int motorPIDSlot = PIDSlots.DEFAULT_POSITION.ordinal();
    public boolean motorConnected = false;
    public AngularVelocity motorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration motorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage motorAppliedVolts = Volts.zero();
    public Current motorCurrent = Amps.zero();
  }

  /**
   * Updates the turret input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(DeflectorIO.DeflectorIOInputs inputs) {}

  /**
   * Sets the turret to run to a target position using closed-loop control.
   *
   * @param position The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  default void setDeflectorPosition(Angle position, PIDSlots pidSlot) {}

  /**
   * Runs the turret using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setDeflectorOpenLoop(double percentOutput) {}

  /** Stops all turret motor movement. */
  default void stop() {}
}
