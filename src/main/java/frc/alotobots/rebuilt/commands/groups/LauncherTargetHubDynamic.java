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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPositionAtVelocity;
import java.util.function.Supplier;

public class LauncherTargetHubDynamic extends ParallelCommandGroup {
  public LauncherTargetHubDynamic(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LaunchCalculator launchCalculator) {
    addCommands(
        new DeflectorFollowPosition(
            deflectorSubsystem, () -> launchCalculator.getHubTargetParameters().deflectorAngle()),
        new DefaultShooterRunAtVelocity(
            shooterSubsystem, () -> launchCalculator.getHubTargetParameters().shooterVelocity()),
        new TurretFollowPositionAtVelocity(
            turretSubsystem,
            () -> launchCalculator.getHubTargetParameters().turretAngleFieldRelative().getMeasure(),
            () -> launchCalculator.getHubTargetParameters().turretVelocity()),
        new RunCommand(launchCalculator::clearHubLaunchingParameters));
  }

  public LauncherTargetHubDynamic(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      LaunchCalculator launchCalculator,
      Supplier<AngularVelocity> shooterVelocityOverride,
      Supplier<Angle> deflectorAngleOverride) {
    addCommands(
        new DeflectorFollowPosition(deflectorSubsystem, deflectorAngleOverride),
        new DefaultShooterRunAtVelocity(shooterSubsystem, shooterVelocityOverride),
        new TurretFollowPositionAtVelocity(
            turretSubsystem,
            () -> launchCalculator.getHubTargetParameters().turretAngleFieldRelative().getMeasure(),
            () -> launchCalculator.getHubTargetParameters().turretVelocity()),
        new RunCommand(launchCalculator::clearHubLaunchingParameters));
  }
}
