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
package frc.alotobots.rebuilt.subsystems.launcher.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.rebuilt.FieldConstants.Hub;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretAngleCalculations {
  public record CartesianCoordinates(double x, double y) {}

  public record PolarCoordinates(double radius, double angle) {}

  private SwerveDriveSubsystem swerveDriveSubsystem;
  private TurretSubsystem turretSubsystem;

  public TurretAngleCalculations(
      SwerveDriveSubsystem swerveDriveSubsystem, TurretSubsystem turretSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  // 15cm forward
  // 5cm right
  @AutoLogOutput
  public Rotation2d stationaryTurretAngleCalculations() {
    var hubLocationBlue = Hub.topCenterPoint;
    var hubLocationRed = Hub.oppTopCenterPoint;
    var robotPose = swerveDriveSubsystem.getPose();

    double deltaX;
    double deltaY;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      deltaX =
          hubLocationRed.getX()
              - robotPose.getX()
              + TurretTalonFXSConstants.ROBOT_TO_TURRET_OFFSET_X;
      deltaY =
          hubLocationRed.getY()
              - robotPose.getY()
              + TurretTalonFXSConstants.TURRET_TO_TURRET_OFFSET_Y;
    } else {
      deltaX =
          hubLocationBlue.getX()
              - robotPose.getX()
              + TurretTalonFXSConstants.ROBOT_TO_TURRET_OFFSET_X;
      deltaY =
          hubLocationBlue.getY()
              - robotPose.getY()
              + TurretTalonFXSConstants.TURRET_TO_TURRET_OFFSET_Y;
    }

    var polarCoordinates = cartesianToPolar(deltaX, deltaY);

    var targetAngle = new Rotation2d(polarCoordinates.angle);
    var targetAngleAdjusted = targetAngle.minus(swerveDriveSubsystem.getPose().getRotation());

    return targetAngleAdjusted;
  }

  @AutoLogOutput
  public PolarCoordinates cartesianToPolar(double x, double y) {
    double radius = Math.hypot(x, y);
    double angle = Math.atan2(y, x);

    return new PolarCoordinates(radius, angle);
  }
}
