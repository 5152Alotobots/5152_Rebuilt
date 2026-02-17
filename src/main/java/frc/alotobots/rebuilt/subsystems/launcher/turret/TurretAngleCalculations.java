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

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.rebuilt.FieldConstants.Hub;
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
    // TODO implement side flipping
    // TODO implement better offset adjustment
    var hubLocation = Hub.topCenterPoint;
    var robotPose = swerveDriveSubsystem.getPose();
    var deltaX = hubLocation.getX() - robotPose.getX() - .05;
    var deltaY = hubLocation.getY() - robotPose.getY() + .15;

    var polarCoordinates = cartesianToPolar(deltaX, deltaY);
    var targetAngle = new Rotation2d(polarCoordinates.angle);

    return targetAngle;
  }

  @AutoLogOutput
  public Rotation2d turretDriveAdjustedAngle() {
    var turretRotation = new Rotation2d(turretSubsystem.getCurrentAngle().in(Radians));
    turretRotation.plus(swerveDriveSubsystem.getPose().getRotation());
    return turretRotation;
  }

  @AutoLogOutput
  public PolarCoordinates cartesianToPolar(double x, double y) {
    double radius = Math.hypot(x, y);
    double angle = Math.atan2(y, x);

    return new PolarCoordinates(radius, angle);
  }
}
