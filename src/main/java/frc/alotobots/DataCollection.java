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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.rebuilt.commands.groups.LauncherTargetHubDynamicAndShoot;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DataCollection {
  @AutoLogOutput(key = "DataCollection/shooterVelocityOverride")
  private AngularVelocity shooterVelocityOverride = RotationsPerSecond.of(30);

  @AutoLogOutput(key = "DataCollection/deflectorAngleOverride")
  private Angle deflectorAngleOverride = DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE;

  public DataCollection(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      RollerSubsystem rollerSubsystem,
      LaunchCalculator launchCalculator) {

    OI.shootData.whileTrue(
        new LauncherTargetHubDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator,
            () -> shooterVelocityOverride,
            () -> deflectorAngleOverride));
    OI.deflectorDownData.onTrue(
        new InstantCommand(
            () -> deflectorAngleOverride = deflectorAngleOverride.plus(Degrees.of(2.5))));
    OI.deflectorUpData.onTrue(
        new InstantCommand(
            () -> deflectorAngleOverride = deflectorAngleOverride.minus(Degrees.of(2.5))));
    OI.rpmDownData.onTrue(
        new InstantCommand(
            () ->
                shooterVelocityOverride =
                    shooterVelocityOverride.minus(RotationsPerSecond.of(2.5))));
    OI.rpmUpData.onTrue(
        new InstantCommand(
            () ->
                shooterVelocityOverride =
                    shooterVelocityOverride.plus(RotationsPerSecond.of(2.5))));
    OI.logData.onTrue(
        new InstantCommand(launchCalculator::clearHubLaunchingParameters)
            .andThen(
                new InstantCommand(
                    () ->
                        Logger.recordOutput(
                            "DataCollection/data",
                            String.format(
                                "Turret Deg: %f, Deflector Deg: %f, RPS: %f, Distance: %f",
                                turretSubsystem.getCurrentAngle().in(Degrees),
                                deflectorAngleOverride.in(Degrees),
                                shooterVelocityOverride.in(RotationsPerSecond),
                                launchCalculator
                                    .getHubTargetParameters()
                                    .dataCollectionDebugDistance()
                                    .in(Meters))))));
  }
}
