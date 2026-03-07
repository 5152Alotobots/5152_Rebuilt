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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.alotobots.rebuilt.subsystems.climber.ClimberSubsystem;
import frc.alotobots.rebuilt.subsystems.climber.commands.ClimberRunToExtension;
import frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretRunToPosition;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants;

public class ClimberUpWithTurretSafety extends ParallelCommandGroup {
  public ClimberUpWithTurretSafety(
      ClimberSubsystem climberSubsystem, TurretSubsystem turretSubsystem) {
    addCommands(
        new TurretRunToPosition(
            turretSubsystem, TurretConstants.Setpoints.SAFETY_TURRET_ANGLE_CLIMB_UP).withTimeout(Seconds.of(1)),
        new ClimberRunToExtension(climberSubsystem, ClimberConstants.Limits.MAX_CLIMB_EXTENSION));
  }
}
