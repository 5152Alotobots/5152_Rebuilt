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

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretAngleCalculations;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.util.Elastic;
import frc.alotobots.util.Elastic.ElasticNotification;
import frc.alotobots.util.NotificationPresets.SwerveDrive;

import org.littletonrobotics.junction.Logger;

public class RunTurretToTarget extends Command {
  private TurretSubsystem turretSubsystem;
  private SwerveDriveSubsystem swerveDriveSubsystem;

  public RunTurretToTarget(TurretSubsystem turretSubsystem, SwerveDriveSubsystem swerveDriveSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    var targetAngle = TurretAngleCalculations.stationaryTurretAngleCalculations(swerveDriveSubsystem.getPose());
    turretSubsystem.runToTargetAngle(targetAngle.getMeasure());
    Logger.recordOutput("Turret/calculatedTargetAngle", targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();

    if (interrupted) {
      Elastic.sendAlert(
          new ElasticNotification()
              .withDisplaySeconds(4)
              .withLevel(Elastic.ElasticNotification.NotificationLevel.INFO)
              .withTitle("Turret Command Interrupted")
              .withDescription("The turret command was interrupted and has stopped."));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
