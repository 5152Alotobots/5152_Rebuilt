/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.turret.TurretAngleCalculations;
import org.littletonrobotics.junction.Logger;

public class RunTurretToTarget extends Command {
  private TurretAngleCalculations turretAngleCalculations;

  public RunTurretToTarget(TurretAngleCalculations turretAngleCalculations) {
    this.turretAngleCalculations = turretAngleCalculations;
  }

  @Override
  public void execute() {
    var targetAngle = turretAngleCalculations.stationaryTurretAngleCalculations();
    Logger.recordOutput("Turret/calculatedTargetAngle", targetAngle);
  }
}
