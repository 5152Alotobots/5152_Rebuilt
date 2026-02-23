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

/**
 * Utility class that computes the desired turret angle based on robot pose and target geometry.
 *
 * <p>Contains both a stationary (odometry-only) angle calculation and a drive-adjusted variant that
 * compensates for robot rotation. All geometry is expressed in field-relative coordinates.
 */
public class TurretAngleCalculations {
  /**
   * Represents a point in two-dimensional Cartesian space.
   *
   * @param x The x coordinate
   * @param y The y coordinate
   */
  public record CartesianCoordinates(double x, double y) {}

  /**
   * Represents a point in two-dimensional polar space.
   *
   * @param radius The radial distance from the origin
   * @param angle The angle from the positive x-axis in radians
   */
  public record PolarCoordinates(double radius, double angle) {}

  private SwerveDriveSubsystem swerveDriveSubsystem;
  private TurretSubsystem turretSubsystem;

  /**
   * Creates a new TurretAngleCalculations helper.
   *
   * @param swerveDriveSubsystem The swerve drive subsystem used to obtain the robot's field pose
   * @param turretSubsystem The turret subsystem used to obtain the current turret angle
   */
  public TurretAngleCalculations(
      SwerveDriveSubsystem swerveDriveSubsystem, TurretSubsystem turretSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  /**
   * Calculates the field-relative turret angle required to aim at the hub from the robot's current
   * pose, assuming the robot is stationary.
   *
   * <p>Applies a fixed offset to account for the turret's physical mounting position relative to
   * the robot center (15 cm forward, 5 cm right).
   *
   * @return The target turret angle as a {@link Rotation2d}
   */
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

  /**
   * Returns the turret angle adjusted for the robot's current field-relative heading.
   *
   * <p>Adds the robot's pose rotation to the current turret angle so that callers can reason about
   * the turret direction in field-relative terms.
   *
   * @return The drive-adjusted turret angle as a {@link Rotation2d}
   */
  @AutoLogOutput
  public Rotation2d turretDriveAdjustedAngle() {
    var turretRotation = new Rotation2d(turretSubsystem.getCurrentAngle().in(Radians));
    turretRotation.plus(swerveDriveSubsystem.getPose().getRotation());
    return turretRotation;
  }

  /**
   * Converts Cartesian coordinates to polar coordinates.
   *
   * @param x The x component of the Cartesian coordinate
   * @param y The y component of the Cartesian coordinate
   * @return A {@link PolarCoordinates} record containing the radius and angle (in radians, measured
   *     from the positive x-axis)
   */
  @AutoLogOutput
  public PolarCoordinates cartesianToPolar(double x, double y) {
    double radius = Math.hypot(x, y);
    double angle = Math.atan2(y, x);

    return new PolarCoordinates(radius, angle);
  }
}
