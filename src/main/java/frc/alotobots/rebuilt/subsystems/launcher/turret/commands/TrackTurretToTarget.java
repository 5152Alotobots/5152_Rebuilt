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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretAngleCalculations;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.util.Elastic;
import frc.alotobots.util.Elastic.ElasticNotification;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TrackTurretToTarget extends Command {
  private TurretSubsystem turretSubsystem;
  private Supplier<Pose2d> robotPose;

  public TrackTurretToTarget(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPose) {
    this.turretSubsystem = turretSubsystem;
    this.robotPose = robotPose;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    var targetAngle = TurretAngleCalculations.stationaryTurretAngleCalculations(robotPose.get());
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
    return false; // This command runs until interrupted
    // TODO: CONSIDER: Adding a condition to end the command when the turret is within a certain
    // threshold of the target angle
  }
}
