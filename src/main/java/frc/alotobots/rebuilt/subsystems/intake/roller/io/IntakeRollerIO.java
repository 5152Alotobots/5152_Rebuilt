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
package frc.alotobots.rebuilt.subsystems.intake.roller.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

  /** Data structure for inputs from intake roller hardware. */
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean intakeRollerMotorConnected = false;
    public AngularVelocity intakeRollerMotorVelocity = RotationsPerSecond.zero();
    public Voltage intakeRollerMotorVolts = Volts.zero();
    public Current intakeRollerMotorCurrent = Amps.zero();
  }

  /**
   * Updates the intake roller input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(IntakeRollerIOInputs inputs) {}
  
  /**
   * Runs the intake rollers using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setIntakeRollerOpenLoop(double percentOutput) {}

  /** Stops all intake roller motor movement. */
  default void stop() {}
}
