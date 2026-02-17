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
package frc.alotobots.rebuilt.subsystems.intake.extendo.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;

public class IntakeExtendoRunToExtension extends Command {

  IntakeExtendoSubsystem intakeExtendoSubsystem;
  Distance targetExtension;

  public IntakeExtendoRunToExtension(
      IntakeExtendoSubsystem intakeExtendoSubsystem, Distance targetExtension) {
    this.intakeExtendoSubsystem = intakeExtendoSubsystem;
    this.targetExtension = targetExtension;
    addRequirements(intakeExtendoSubsystem);
  }

  @Override
  public void initialize() {
    intakeExtendoSubsystem.runToTargetPosition(targetExtension);
  }

  @Override
  public void execute() {
    // Position control handled by subsystem
  }

  @Override
  public void end(boolean interrupted) {
    intakeExtendoSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return intakeExtendoSubsystem.isAtTargetExtension();
  }
}
