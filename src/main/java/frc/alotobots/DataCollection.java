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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.rebuilt.commands.groups.LauncherTargetHubDynamicAndShoot;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DataCollection {
  @AutoLogOutput(key = "DataCollection/targetrpm")
  private AngularVelocity shooterVelocity = RotationsPerSecond.of(30);

  public DataCollection(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      LaunchCalculator launchCalculator) {
    OI.shootData.whileTrue(
        new LauncherTargetHubDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            launchCalculator));
    OI.deflectorDownData.onTrue(
        new DeflectorFollowPosition(
            deflectorSubsystem, () -> deflectorSubsystem.getCurrentAngle().plus(Degrees.of(5))));
    OI.deflectorUpData.onTrue(
        new DeflectorFollowPosition(
            deflectorSubsystem, () -> deflectorSubsystem.getCurrentAngle().minus(Degrees.of(5))));
    OI.rpmDownData.onTrue(
        new InstantCommand(
            () -> shooterVelocity = shooterVelocity.minus(RotationsPerSecond.of(2.5))));
    OI.rpmUpData.onTrue(
        new InstantCommand(
            () -> shooterVelocity = shooterVelocity.plus(RotationsPerSecond.of(2.5))));
    OI.logData.onTrue(
        new InstantCommand(launchCalculator::clearLaunchingParameters)
            .andThen(
                new InstantCommand(
                    () ->
                        Logger.recordOutput(
                            "DataCollection/data",
                            String.format(
                                "Turret Deg: %f, Deflector Deg: %f, RPS: %f, Distance: %f",
                                turretSubsystem.getCurrentAngle().in(Degrees),
                                deflectorSubsystem.getCurrentAngle().in(Degrees),
                                shooterVelocity.in(RotationsPerSecond),
                                launchCalculator
                                    .getParameters()
                                    .dataCollectionDebugDistance()
                                    .in(Meters))))));
  }
}
