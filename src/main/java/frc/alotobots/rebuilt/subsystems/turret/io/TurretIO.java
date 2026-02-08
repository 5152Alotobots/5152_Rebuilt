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
package frc.alotobots.rebuilt.subsystems.turret.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  enum PIDSlots {
    DEFAULT_POSITION,
  }

  /** Data structure for inputs from turret hardware. */
  @AutoLog
  public static class TurretIOInputs {
    public PIDSlots turretMotorPidSlot = PIDSlots.DEFAULT_POSITION;
    public boolean turretMotorConnected = false;
    public Angle turretMotorPosition = Rotations.zero();
    public AngularVelocity turretMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration turretMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage turretMotorVolts = Volts.zero();
    public Current turretMotorCurrent = Amps.zero();
    public ControlModeValue turretMotorControlMode = null;

    public boolean ccwLimit = false;
    public boolean cwLimit = false;
  }

  /**
   * Updates the turret input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Sets the turret to run to a target position using closed-loop control.
   *
   * @param position The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  default void setTurretPosition(Angle position, PIDSlots pidSlot) {}

  default void setTurretPosition(Angle position) {}

  /**
   * Runs the turret using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setTurretOpenLoop(double percentOutput) {}

  default void setTurretVoltageOut(Voltage voltageOutput) {}

  /** Stops all turret motor movement. */
  default void stop() {}
}
