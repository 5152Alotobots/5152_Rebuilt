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

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.commands.DefaultRollerRunOpenLoop;
import frc.alotobots.rebuilt.subsystems.roller.constants.RollerConstants;

public class LauncherShoot extends SequentialCommandGroup {
  public LauncherShoot(
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      BeltSubsystem beltSubsystem,
      RollerSubsystem rollerSubsystem,
      KickerSubsystem kickerSubsystem) {
    addCommands(
        new ParallelCommandGroup(
            // Kicker runs forward when ready to shoot, backward otherwise
            new DefaultKickerRunAtVelocity(
                kickerSubsystem,
                () ->
                    (shooterSubsystem.isAtTargetVelocity() && !turretSubsystem.isFlipping())
                        ? KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY
                        : KickerConstants.Setpoints.LOAD_OUT_OF_SHOOTER_VELOCITY),
            // Belt only runs when ready to shoot
            new DefaultBeltRunAtVelocity(
                beltSubsystem,
                () ->
                    (shooterSubsystem.isAtTargetVelocity() && !turretSubsystem.isFlipping())
                        ? BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY
                        : RotationsPerSecond.of(0)),
            // Roller only jostles when ready to shoot
            new DefaultRollerRunOpenLoop(
                rollerSubsystem,
                () ->
                    (shooterSubsystem.isAtTargetVelocity() && !turretSubsystem.isFlipping())
                        ? RollerConstants.Setpoints.OpenLoop.JOSTLE
                        : 0.0)));
  }
}
