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
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import java.util.function.Supplier;

public class LauncherTargetPassingDynamicAndShoot extends ParallelCommandGroup {

  public LauncherTargetPassingDynamicAndShoot(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      RollerSubsystem rollerSubsystem,
      LaunchCalculator launchCalculator) {
    addCommands(
        new LauncherTargetPassingDynamic(
            deflectorSubsystem, shooterSubsystem, turretSubsystem, launchCalculator),
        new LauncherShoot(shooterSubsystem, beltSubsystem, rollerSubsystem, kickerSubsystem));
  }

  public LauncherTargetPassingDynamicAndShoot(
      DeflectorSubsystem deflectorSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      RollerSubsystem rollerSubsystem,
      LaunchCalculator launchCalculator,
      Supplier<AngularVelocity> shooterVelocityOverride,
      Supplier<Angle> deflectorAngleOverride) {
    addCommands(
        new LauncherTargetPassingDynamic(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            launchCalculator,
            shooterVelocityOverride,
            deflectorAngleOverride),
        new LauncherShoot(shooterSubsystem, beltSubsystem, rollerSubsystem, kickerSubsystem));
  }
}
