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
package frc.alotobots.rebuilt.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.LOOP_PERIOD;
import static frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculatorConstants.*;
import static frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculatorConstants.Maps.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.rebuilt.FieldConstants;
import frc.alotobots.util.GeomUtil;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates launch parameters accounting for robot motion and target position.
 *
 * @author 5152 Alotobots, adapted from 6328 Littleton Robotics
 */
@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotRelativeVelocitySupplier;
  private final Supplier<ChassisSpeeds> robotFieldVelocitySupplier;

  /** Constructs a LaunchCalculator with suppliers for robot pose and velocity. */
  public LaunchCalculator(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotRelativeVelocitySupplier,
      Supplier<ChassisSpeeds> robotFieldVelocitySupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotRelativeVelocitySupplier = robotRelativeVelocitySupplier;
    this.robotFieldVelocitySupplier = robotFieldVelocitySupplier;
  }

  /** Filters turret angle derivative to smooth velocity estimation. */
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  /** Filters deflector angle derivative to smooth velocity estimation. */
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  /** Turret angle from the previous loop cycle, used for velocity estimation. */
  private Rotation2d lastTurretAngle = null;

  /** Deflector angle from the previous loop cycle, used for velocity estimation. */
  private Angle lastHoodAngle = null;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngleFieldRelative,
      AngularVelocity turretVelocity,
      Angle deflectorAngle,
      AngularVelocity deflectorVelocity,
      AngularVelocity shooterVelocity,
      // TODO: Remove this temporary value.
      Distance dataCollectionDebugDistance) {}

  /** Cached parameters for the current loop cycle, null if not yet computed. */
  private LaunchingParameters latestParameters = null;

  /**
   * Returns the cached launching parameters for this loop cycle, computing them if not yet
   * calculated.
   */
  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d forwardEstimatedPose = robotPoseSupplier.get();
    ChassisSpeeds robotRelativeVelocity = robotRelativeVelocitySupplier.get();
    forwardEstimatedPose =
        forwardEstimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond
                    * LAUNCH_CALCULATOR_SUBSYSTEM_DELAY.in(Seconds),
                robotRelativeVelocity.vyMetersPerSecond
                    * LAUNCH_CALCULATOR_SUBSYSTEM_DELAY.in(Seconds),
                robotRelativeVelocity.omegaRadiansPerSecond
                    * LAUNCH_CALCULATOR_SUBSYSTEM_DELAY.in(Seconds)));

    // Calculate distance from turret to target
    boolean shouldFlip;
    if (DriverStation.getAlliance().isEmpty()) {
      DriverStation.reportWarning(
          "No alliance data available to calculate distance to turret target. Defaulting to blue.",
          false);
      shouldFlip = false;
    } else {
      shouldFlip = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    // Flip if necessary
    Translation2d target;
    if (shouldFlip) {
      target = FlippingUtil.flipFieldPosition(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else {
      target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    }

    // Calculate where the turret will be in the future and its distance from the hub
    Pose2d turretPosition = forwardEstimatedPose.transformBy(ROBOT_TO_TURRET);
    Distance turretToTargetDistance =
        Meters.of(target.getDistance(turretPosition.getTranslation()));

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = robotFieldVelocitySupplier.get();
    Angle robotAngle = Radians.of(forwardEstimatedPose.getRotation().getRadians());

    LinearVelocity turretLinearVelocityX =
        MetersPerSecond.of(
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (ROBOT_TO_TURRET.getY() * Math.cos(robotAngle.in(Radians))
                        - ROBOT_TO_TURRET.getX() * Math.sin(robotAngle.in(Radians))));
    LinearVelocity turretLinearVelocityY =
        MetersPerSecond.of(
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (ROBOT_TO_TURRET.getX() * Math.cos(robotAngle.in(Radians))
                        - ROBOT_TO_TURRET.getY() * Math.sin(robotAngle.in(Radians))));

    // Account for imparted velocity by robot (turret) to offset
    Time timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    Distance lookaheadTurretToTargetDistance = turretToTargetDistance;

    // The time of flight depends on distance, but distance depends on time of flight (circular
    // dependency).
    // We break this by iteratively refining our guess, each iteration uses the previous time of
    // flight
    // to estimate where the robot will be, giving a better distance, giving a better time of
    // flight, etc.
    for (int i = 0; i < 20; i++) {
      timeOfFlight = (Time) FUEL_TIME_OF_FLIGHT_MAP.get(lookaheadTurretToTargetDistance);
      Translation2d turretOffsetToLookahead =
          new Translation2d(
              turretLinearVelocityX.in(MetersPerSecond) * timeOfFlight.in(Seconds),
              turretLinearVelocityY.in(MetersPerSecond) * timeOfFlight.in(Seconds));

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(turretOffsetToLookahead),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance =
          Meters.of(target.getDistance(lookaheadPose.getTranslation()));
    }

    // Calculate parameters accounted for imparted velocity and rotate to field relative
    Rotation2d turretFieldRelativeAngle =
        target
            .minus(lookaheadPose.getTranslation())
            .rotateBy(forwardEstimatedPose.getRotation().unaryMinus())
            .getAngle();
    Angle deflectorAngle =
        (Angle) LAUNCHER_DEFLECTOR_ANGLE_MAP.get(lookaheadTurretToTargetDistance);

    // Store historical values
    if (lastTurretAngle == null) {
      lastTurretAngle = turretFieldRelativeAngle;
    }

    if (lastHoodAngle == null) {
      lastHoodAngle = deflectorAngle;
    }

    AngularVelocity turretVelocity =
        RadiansPerSecond.of(
            turretAngleFilter.calculate(
                turretFieldRelativeAngle.minus(lastTurretAngle).getRadians()
                    / LOOP_PERIOD.in(Seconds)));
    AngularVelocity deflectorVelocity =
        RadiansPerSecond.of(
            hoodAngleFilter.calculate(
                (deflectorAngle.minus(lastHoodAngle)).in(Radians) / LOOP_PERIOD.in(Seconds)));

    lastTurretAngle = turretFieldRelativeAngle;
    lastHoodAngle = deflectorAngle;

    AngularVelocity shooterVelocity =
        (AngularVelocity) LAUNCHER_SHOOTER_VELOCITY_MAP.get(lookaheadTurretToTargetDistance);

    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance.gte(MINIMUM_SHOOTING_DISTANCE)
                && lookaheadTurretToTargetDistance.lte(MAXIMUM_SHOOTING_DISTANCE),
            turretFieldRelativeAngle,
            turretVelocity,
            deflectorAngle,
            deflectorVelocity,
            shooterVelocity,
            lookaheadTurretToTargetDistance);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput("LaunchCalculator/Perameters", latestParameters);
    return latestParameters;
  }

  /**
   * Invalidates the cached parameters, forcing recomputation on the next {@link #getParameters()}
   * call. Should be called once per loop cycle.
   */
  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
