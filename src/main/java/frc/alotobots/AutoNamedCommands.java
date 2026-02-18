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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.Map;

/** Registers and manages named commands for autonomous routines. */
public class AutoNamedCommands {
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** Constructs command registration manager with required subsystems. */
  public AutoNamedCommands(SwerveDriveSubsystem swerveDriveSubsystem) {

    this.swerveDriveSubsystem = swerveDriveSubsystem;

    registerCommands();
  }

  /** Registers all available autonomous commands with PathPlanner. */
  public void registerCommands() {
    Map<String, Command> commands = new HashMap<>();

    // Auto states
    commands.put("AutoStopWithX", new InstantCommand(swerveDriveSubsystem::stopWithX));

    NamedCommands.registerCommands(commands);
  }
}
