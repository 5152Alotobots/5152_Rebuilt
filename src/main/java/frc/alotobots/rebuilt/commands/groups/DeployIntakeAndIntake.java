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
package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.extendo.commands.IntakeExtendoRunToExtension;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants;
import frc.alotobots.rebuilt.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.roller.commands.IntakeRollerIntake;
import frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants;

public class DeployIntakeAndIntake extends ParallelCommandGroup {
  IntakeExtendoSubsystem intakeExtendoSubsystem;
  IntakeRollerSubsystem intakeRollerSubsystem;

  public DeployIntakeAndIntake(
      IntakeExtendoSubsystem intakeExtendoSubsystem, IntakeRollerSubsystem intakeRollerSubsystem) {
    this.intakeExtendoSubsystem = intakeExtendoSubsystem;
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    addCommands(
        new IntakeExtendoRunToExtension(
            intakeExtendoSubsystem, IntakeExtendoConstants.Setpoints.DEPLOYED),
        new IntakeRollerIntake(
            intakeRollerSubsystem,
            () -> IntakeRollerConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE));
  }
}
