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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;

public class DeflectorRunToPosition extends Command {
  private final DeflectorSubsystem deflectorSubsystem;
  private final Angle angle;

  public DeflectorRunToPosition(DeflectorSubsystem deflectorSubsystem, Angle angle) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.angle = angle;

    addRequirements(deflectorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    deflectorSubsystem.runToTargetAngle(angle);
    ;
  }

  @Override
  public void end(boolean interrupted) {
    deflectorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
