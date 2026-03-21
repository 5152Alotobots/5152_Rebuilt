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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPositionAtVelocity;

public class LauncherTargetHubDynamicAndShoot extends SequentialCommandGroup {
  public LauncherTargetHubDynamicAndShoot(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      LaunchCalculator launchCalculator) {
    addCommands(
        new ParallelCommandGroup(
                new DeflectorFollowPosition(
                    deflectorSubsystem,
                    () -> launchCalculator.getHubTargetParameters().deflectorAngle()),
                new DefaultShooterRunAtVelocity(
                    shooterSubsystem,
                    () -> launchCalculator.getHubTargetParameters().shooterVelocity()),
                new TurretFollowPositionAtVelocity(
                    turretSubsystem,
                    () ->
                        launchCalculator
                            .getHubTargetParameters()
                            .turretAngleFieldRelative()
                            .getMeasure(),
                    () -> launchCalculator.getHubTargetParameters().turretVelocity()),
                new RunCommand(launchCalculator::clearHubLaunchingParameters))
            .deadlineFor(
                new SequentialCommandGroup(
                    // Wait for shooter to spin up
                    new WaitUntilCommand(shooterSubsystem::isAtTargetVelocity),
                    // Then feed
                    new ParallelCommandGroup(
                        new DefaultBeltRunAtVelocity(
                            beltSubsystem,
                            () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                        new DefaultKickerRunAtVelocity(
                            kickerSubsystem,
                            () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY)))));
  }

  public LauncherTargetHubDynamicAndShoot(
          DeflectorSubsystem deflectorSubsystem,
          ShooterSubsystem shooterSubsystem,
          TurretSubsystem turretSubsystem,
          KickerSubsystem kickerSubsystem,
          BeltSubsystem beltSubsystem,
          LaunchCalculator launchCalculator,
          Trigger releaseTrigger) {
    addCommands(
            new ParallelCommandGroup(
                    new DeflectorFollowPosition(
                            deflectorSubsystem,
                            () -> launchCalculator.getHubTargetParameters().deflectorAngle()),
                    new DefaultShooterRunAtVelocity(
                            shooterSubsystem,
                            () -> launchCalculator.getHubTargetParameters().shooterVelocity()),
                    new TurretFollowPositionAtVelocity(
                            turretSubsystem,
                            () ->
                                    launchCalculator
                                            .getHubTargetParameters()
                                            .turretAngleFieldRelative()
                                            .getMeasure(),
                            () -> launchCalculator.getHubTargetParameters().turretVelocity()),
                    new RunCommand(launchCalculator::clearHubLaunchingParameters))
                    .deadlineFor(
                            new SequentialCommandGroup(
                                    // Wait for shooter to spin up
                                    new WaitUntilCommand(shooterSubsystem::isAtTargetVelocity),
                                    // Then feed
                                    new ParallelCommandGroup(
                                            new DefaultBeltRunAtVelocity(
                                                    beltSubsystem,
                                                    () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                                            new DefaultKickerRunAtVelocity(
                                                    kickerSubsystem,
                                                    () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY))
                                            .onlyWhile(releaseTrigger)
                            )));
  }
}
