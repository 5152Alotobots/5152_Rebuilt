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
package frc.alotobots.rebuilt.subsystems.launcher.turret.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;

public class TurretRunToPosition extends Command {
  private final TurretSubsystem turretSubsystem;
  private final Angle targetAngle;

  public TurretRunToPosition(TurretSubsystem turretSubsystem, Angle targetAngle) {
    this.turretSubsystem = turretSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.runToTargetAngle(targetAngle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.isAtTargetAngle();
  }
}
