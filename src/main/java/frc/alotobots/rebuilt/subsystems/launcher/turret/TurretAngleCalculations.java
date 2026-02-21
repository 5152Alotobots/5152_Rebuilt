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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.rebuilt.FieldConstants.Hub;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretAngleCalculations {
  public record CartesianCoordinates(double x, double y) {}

  public record PolarCoordinates(double radius, double angle) {}

  // 15cm forward
  // 5cm right
  @AutoLogOutput
  public static Rotation2d stationaryTurretAngleCalculations(Pose2d robotPose) {
    var hubLocationBlue = Hub.topCenterPoint;
    var hubLocationRed = Hub.oppTopCenterPoint;

    double deltaX;
    double deltaY;

    robotPose = robotPose.transformBy(new Transform2d(
      TurretTalonFXSConstants.ROBOT_TO_TURRET_OFFSET_X, 
      TurretTalonFXSConstants.ROBOT_TO_TURRET_OFFSET_Y, 
      new Rotation2d()));

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      deltaX =
          hubLocationRed.getX()
              - robotPose.getX();
      deltaY =
          hubLocationRed.getY()
              - robotPose.getY();
    } else {
      deltaX =
          hubLocationBlue.getX()
              - robotPose.getX();
      deltaY =
          hubLocationBlue.getY()
              - robotPose.getY();
    }

    var polarCoordinates = cartesianToPolar(deltaX, deltaY);

    var targetAngle = new Rotation2d(polarCoordinates.angle);
    var targetAngleAdjusted = targetAngle.minus(robotPose.getRotation());

    return targetAngleAdjusted;
  }

  @AutoLogOutput
  public static PolarCoordinates cartesianToPolar(double x, double y) {
    double radius = Math.hypot(x, y);
    double angle = Math.atan2(y, x);

    return new PolarCoordinates(radius, angle);
  }
}
