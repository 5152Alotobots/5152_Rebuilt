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
package frc.alotobots.rebuilt.subsystems.intake.roller.commands;

import static frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.intake.roller.IntakeRollerSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollerEject extends Command {
  private IntakeRollerSubsystem intakeRollerSubsystem;
  private DoubleSupplier input;

  public IntakeRollerEject(IntakeRollerSubsystem intakeRollerSubsystem, DoubleSupplier input) {
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.input = input;
    addRequirements(intakeRollerSubsystem);
  }

  /**
   * Runs the intake motors at the supplied speed to push outward, clamped to safe limits. Called
   * repeatedly while the command is scheduled.
   */
  @Override
  public void execute() {
    double adjustedOutput =
        MathUtil.clamp(-input.getAsDouble(), -MAX_OPEN_LOOP_EJECT_PERCENTAGE, 0);
    intakeRollerSubsystem.runAtPercentOutput(adjustedOutput);
  }

  /**
   * Called when the command ends or is interrupted. Stops the intake motors.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    intakeRollerSubsystem.stop();
  }

  /**
   * Determines if the command has finished. Returns true once a game piece is detected.
   *
   * @return true if a game piece is detected in the intake
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
