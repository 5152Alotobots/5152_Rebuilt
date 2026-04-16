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
                new DefaultBeltRunAtVelocity(
                    beltSubsystem, () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                new DefaultRollerRunOpenLoop(
                    rollerSubsystem, () -> RollerConstants.Setpoints.OpenLoop.JOSTLE),
                new DefaultKickerRunAtVelocity(
                    kickerSubsystem, () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY))
            .onlyWhile(() -> /*shooterSubsystem.isAtTargetVelocity() &&*/ !turretSubsystem.isFlipping())
            .repeatedly());
  }
}
