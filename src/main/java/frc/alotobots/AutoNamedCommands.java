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
package frc.alotobots;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.rebuilt.commands.groups.DeployIntakeAndIntake;
import frc.alotobots.rebuilt.commands.groups.LauncherTargetHubDynamicAndShoot;
import frc.alotobots.rebuilt.commands.groups.LauncherTargetPassingDynamicAndShoot;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.climber.ClimberSubsystem;
import frc.alotobots.rebuilt.subsystems.climber.commands.ClimberRunToExtension;
import frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.extendo.commands.IntakeExtendoRunToExtension;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants;
import frc.alotobots.rebuilt.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import java.util.HashMap;
import java.util.Map;

/** Registers and manages named commands for autonomous routines. */
public class AutoNamedCommands {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final DeflectorSubsystem deflectorSubsystem;
  private final IntakeExtendoSubsystem intakeExtendoSubsystem;
  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final BeltSubsystem beltSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final LaunchCalculator launchCalculator;

  /** Constructs command registration manager with required subsystems. */
  public AutoNamedCommands(
      SwerveDriveSubsystem swerveDriveSubsystem,
      DeflectorSubsystem deflectorSubsystem,
      IntakeExtendoSubsystem intakeExtendoSubsystem,
      IntakeRollerSubsystem intakeRollerSubsystem,
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      RollerSubsystem rollerSubsystem,
      ClimberSubsystem climberSubsystem,
      LaunchCalculator launchCalculator) {

    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.deflectorSubsystem = deflectorSubsystem;
    this.intakeExtendoSubsystem = intakeExtendoSubsystem;
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.beltSubsystem = beltSubsystem;
    this.rollerSubsystem = rollerSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.launchCalculator = launchCalculator;
    registerCommands();
  }

  /** Registers all available autonomous commands with PathPlanner. */
  public void registerCommands() {
    Map<String, Command> commands = new HashMap<>();

    // Auto states
    commands.put(
        "DeployIntakeAndIntake",
        new DeployIntakeAndIntake(intakeExtendoSubsystem, intakeRollerSubsystem));
    commands.put(
        "RetractIntake",
        new IntakeExtendoRunToExtension(
            intakeExtendoSubsystem, IntakeExtendoConstants.Setpoints.STOWED));
    commands.put(
        "ClimbUp",
        new ClimberRunToExtension(climberSubsystem, ClimberConstants.Limits.MIN_CLIMB_EXTENSION));
    commands.put(
        "ClimbDown",
        new ClimberRunToExtension(climberSubsystem, ClimberConstants.Limits.MAX_CLIMB_EXTENSION));
    commands.put(
        "LauncherTargetHubDynamicAndShoot",
        new LauncherTargetHubDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator));
    commands.put(
        "LauncherTargetPassingDynamicAndShoot",
        new LauncherTargetPassingDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator));
    NamedCommands.registerCommands(commands);
  }
}
