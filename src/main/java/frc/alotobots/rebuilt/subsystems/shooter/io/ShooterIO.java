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
package frc.alotobots.rebuilt.subsystems.shooter.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  enum PIDSlots {
    DEFAULT_VELOCITY,
  }

  /** Data structure for inputs from shooter hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public PIDSlots shooterMotorLeftPidSlot = PIDSlots.DEFAULT_VELOCITY;
    public PIDSlots shooterMotorRightPidSlot = PIDSlots.DEFAULT_VELOCITY;

    public boolean shooterMotorLeftConnected = false;
    public boolean shooterMotorRightConnected = false;

    public AngularVelocity shooterMotorLeftVelocity = RotationsPerSecond.zero();
    public AngularVelocity shooterMotorRightVelocity = RotationsPerSecond.zero();

    public AngularAcceleration shooterMotorLeftAcceleration = RotationsPerSecondPerSecond.zero();
    public AngularAcceleration shooterMotorRightAcceleration = RotationsPerSecondPerSecond.zero();

    public Voltage shooterMotorLeftVolts = Volts.zero();
    public Voltage shooterMotorRightVolts = Volts.zero();

    public Current shooterMotorLeftCurrent = Amps.zero();
    public Current shooterMotorRightCurrent = Amps.zero();
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
