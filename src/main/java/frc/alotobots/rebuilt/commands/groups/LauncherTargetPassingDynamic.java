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
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPositionAtVelocity;

public class LauncherTargetPassingDynamic extends ParallelCommandGroup {
  public LauncherTargetPassingDynamic(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LaunchCalculator launchCalculator) {
    addCommands(
        new DeflectorFollowPosition(
            deflectorSubsystem,
            () -> launchCalculator.getPassingTargetParameters().deflectorAngle()),
        new DefaultShooterRunAtVelocity(
            shooterSubsystem,
            () -> launchCalculator.getPassingTargetParameters().shooterVelocity()),
        new TurretFollowPositionAtVelocity(
            turretSubsystem,
            () ->
                launchCalculator
                    .getPassingTargetParameters()
                    .turretAngleFieldRelative()
                    .getMeasure(),
            () -> launchCalculator.getPassingTargetParameters().turretVelocity()),
        new RunCommand(launchCalculator::clearPassingLaunchingParameters));
  }
}
