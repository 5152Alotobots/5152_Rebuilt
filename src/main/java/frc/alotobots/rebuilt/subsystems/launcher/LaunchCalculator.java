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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.rebuilt.FieldConstants;
import frc.alotobots.util.GeomUtil;
import java.util.function.Supplier;

import frc.alotobots.util.UnitInterpolatingMap;
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

  // ── Hub filters ──
  /** Filters hub turret angle derivative to smooth velocity estimation. */
  private final LinearFilter hubTurretAngleFilter =
          LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  /** Filters hub deflector angle derivative to smooth velocity estimation. */
  private final LinearFilter hubDeflectorAngleFilter =
          LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  // ── Passing filters ──
  /** Filters passing turret angle derivative to smooth velocity estimation. */
  private final LinearFilter passingTurretAngleFilter =
          LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  /** Filters passing deflector angle derivative to smooth velocity estimation. */
  private final LinearFilter passingDeflectorAngleFilter =
          LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD.in(Seconds)));

  // ── Hub historical state ──
  /** Hub turret angle from the previous loop cycle, used for velocity estimation. */
  private Rotation2d lastHubTurretAngle = null;

  /** Hub deflector angle from the previous loop cycle, used for velocity estimation. */
  private Angle lastHubDeflectorAngle = null;

  // ── Passing historical state ──
  /** Passing turret angle from the previous loop cycle, used for velocity estimation. */
  private Rotation2d lastPassingTurretAngle = null;

  /** Passing deflector angle from the previous loop cycle, used for velocity estimation. */
  private Angle lastPassingDeflectorAngle = null;

  public record LaunchingParameters(
          boolean isValid,
          Rotation2d turretAngleFieldRelative,
          AngularVelocity turretVelocity,
          Angle deflectorAngle,
          AngularVelocity deflectorVelocity,
          AngularVelocity shooterVelocity,
          // TODO: Remove this temporary value.
          Distance dataCollectionDebugDistance) {}

  /**
   * Intermediate result of the lookahead calculation, containing the refined turret pose and
   * distance to the target after accounting for robot velocity and time of flight.
   */
  private record LookaheadResult(Pose2d lookaheadPose, Distance turretToTargetDistance) {}

  /** Cached hub parameters for the current loop cycle, null if not yet computed. */
  private LaunchingParameters latestHubTargetParameters = null;

  /** Cached passing parameters for the current loop cycle, null if not yet computed. */
  private LaunchingParameters latestPassingTargetParameters = null;

  /**
   * Estimates the robot pose after accounting for subsystem phase delay by forward-projecting the
   * current pose using the robot-relative velocity.
   *
   * @return the forward-estimated pose
   */
  private Pose2d getForwardEstimatedPose() {
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
    return forwardEstimatedPose;
  }

  /**
   * Returns the hub target position, flipped to the correct side of the field based on the current
   * driver station alliance.
   *
   * @return the alliance-corrected hub target translation
   */
  private Translation2d getAllianceCorrectedTarget(Translation2d target) {
    boolean shouldFlip;
    if (DriverStation.getAlliance().isEmpty()) {
      DriverStation.reportWarning(
              "No alliance data available to calculate distance to turret target. Defaulting to blue.",
              false);
      shouldFlip = false;
    } else {
      shouldFlip = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    if (shouldFlip) {
      return FlippingUtil.flipFieldPosition(target);
    } else {
      return target;
    }
  }

  /**
   * Calculates the lookahead pose and distance to a target, accounting for robot velocity and
   * iteratively refining based on time of flight.
   *
   * <p>The time of flight depends on distance, but distance depends on time of flight (circular
   * dependency). This is resolved by iteratively refining: each iteration uses the previous time of
   * flight to estimate where the robot will be, giving a better distance, giving a better time of
   * flight, etc.
   *
   * @param target the field-relative position of the target
   * @param forwardEstimatedPose the robot pose after accounting for phase delay
   * @return the refined lookahead pose and distance to the target
   */
  private LookaheadResult calculateLookahead(
          Translation2d target, Pose2d forwardEstimatedPose, UnitInterpolatingMap<DistanceUnit, TimeUnit> timeOfFlightMap) {
    // Calculate where the turret will be in the future and its distance from the target
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

    // Iteratively refine the lookahead pose and distance
    Pose2d lookaheadPose = turretPosition;
    Distance lookaheadTurretToTargetDistance = turretToTargetDistance;

    for (int i = 0; i < 20; i++) {
      Time timeOfFlight = (Time) timeOfFlightMap.get(lookaheadTurretToTargetDistance);
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

    return new LookaheadResult(lookaheadPose, lookaheadTurretToTargetDistance);
  }

  /**
   * Returns the cached hub launching parameters for this loop cycle, computing them if not yet
   * calculated.
   */
  public LaunchingParameters getHubTargetParameters() {
    if (latestHubTargetParameters != null) {
      return latestHubTargetParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d forwardEstimatedPose = getForwardEstimatedPose();

    // Get alliance-corrected hub target
    Translation2d target = getAllianceCorrectedTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Calculate lookahead pose and distance accounting for robot velocity and time of flight
    LookaheadResult lookahead = calculateLookahead(target, forwardEstimatedPose, FUEL_HUB_TIME_OF_FLIGHT_MAP);
    Pose2d lookaheadPose = lookahead.lookaheadPose();
    Distance lookaheadTurretToTargetDistance = lookahead.turretToTargetDistance();

    // Calculate parameters accounted for imparted velocity and rotate to robot relative
    Rotation2d turretFieldRelativeAngle =
            target
                    .minus(lookaheadPose.getTranslation())
                    .rotateBy(forwardEstimatedPose.getRotation().unaryMinus())
                    .getAngle();
    Angle deflectorAngle =
            (Angle) LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.get(lookaheadTurretToTargetDistance);

    // Store historical values
    if (lastHubTurretAngle == null) {
      lastHubTurretAngle = turretFieldRelativeAngle;
    }

    if (lastHubDeflectorAngle == null) {
      lastHubDeflectorAngle = deflectorAngle;
    }

    AngularVelocity turretVelocity =
            RadiansPerSecond.of(
                    hubTurretAngleFilter.calculate(
                            turretFieldRelativeAngle.minus(lastHubTurretAngle).getRadians()
                                    / LOOP_PERIOD.in(Seconds)));
    AngularVelocity deflectorVelocity =
            RadiansPerSecond.of(
                    hubDeflectorAngleFilter.calculate(
                            (deflectorAngle.minus(lastHubDeflectorAngle)).in(Radians) / LOOP_PERIOD.in(Seconds)));

    lastHubTurretAngle = turretFieldRelativeAngle;
    lastHubDeflectorAngle = deflectorAngle;

    AngularVelocity shooterVelocity =
            (AngularVelocity) LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.get(lookaheadTurretToTargetDistance);

    latestHubTargetParameters =
            new LaunchingParameters(
                    lookaheadTurretToTargetDistance.gte(MINIMUM_HUB_SHOOTING_DISTANCE)
                            && lookaheadTurretToTargetDistance.lte(MAXIMUM_HUB_SHOOTING_DISTANCE),
                    turretFieldRelativeAngle,
                    turretVelocity,
                    deflectorAngle,
                    deflectorVelocity,
                    shooterVelocity,
                    lookaheadTurretToTargetDistance);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/Hub/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/Hub/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput("LaunchCalculator/Hub/Parameters", latestHubTargetParameters);
    return latestHubTargetParameters;
  }

  /**
   * Returns the cached passing launching parameters for this loop cycle, computing them if not yet
   * calculated.
   */
  public LaunchingParameters getPassingTargetParameters() {
    if (latestPassingTargetParameters != null) {
      return latestPassingTargetParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d forwardEstimatedPose = getForwardEstimatedPose();

    // Get alliance-corrected passing target
    Translation2d target = getAllianceCorrectedTarget(forwardEstimatedPose.getTranslation().nearest(PASSING_TARGET_OPTIONS));

    // Calculate lookahead pose and distance accounting for robot velocity and time of flight
    LookaheadResult lookahead = calculateLookahead(target, forwardEstimatedPose, FUEL_PASSING_TIME_OF_FLIGHT_MAP);
    Pose2d lookaheadPose = lookahead.lookaheadPose();
    Distance lookaheadTurretToTargetDistance = lookahead.turretToTargetDistance();

    // Calculate parameters accounted for imparted velocity and rotate to robot relative
    Rotation2d turretFieldRelativeAngle =
            target
                    .minus(lookaheadPose.getTranslation())
                    .rotateBy(forwardEstimatedPose.getRotation().unaryMinus())
                    .getAngle();
    Angle deflectorAngle =
            (Angle) LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.get(lookaheadTurretToTargetDistance);

    // Store historical values
    if (lastPassingTurretAngle == null) {
      lastPassingTurretAngle = turretFieldRelativeAngle;
    }

    if (lastPassingDeflectorAngle == null) {
      lastPassingDeflectorAngle = deflectorAngle;
    }

    AngularVelocity turretVelocity =
            RadiansPerSecond.of(
                    passingTurretAngleFilter.calculate(
                            turretFieldRelativeAngle.minus(lastPassingTurretAngle).getRadians()
                                    / LOOP_PERIOD.in(Seconds)));
    AngularVelocity deflectorVelocity =
            RadiansPerSecond.of(
                    passingDeflectorAngleFilter.calculate(
                            (deflectorAngle.minus(lastPassingDeflectorAngle)).in(Radians) / LOOP_PERIOD.in(Seconds)));

    lastPassingTurretAngle = turretFieldRelativeAngle;
    lastPassingDeflectorAngle = deflectorAngle;

    AngularVelocity shooterVelocity =
            (AngularVelocity) LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.get(lookaheadTurretToTargetDistance);

    latestPassingTargetParameters =
            new LaunchingParameters(
                    lookaheadTurretToTargetDistance.gte(MINIMUM_PASSING_SHOOTING_DISTANCE)
                            && lookaheadTurretToTargetDistance.lte(MAXIMUM_PASSING_SHOOTING_DISTANCE),
                    turretFieldRelativeAngle,
                    turretVelocity,
                    deflectorAngle,
                    deflectorVelocity,
                    shooterVelocity,
                    lookaheadTurretToTargetDistance);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/Passing/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/Passing/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput("LaunchCalculator/Passing/Parameters", latestPassingTargetParameters);
    return latestPassingTargetParameters;
  }

  /**
   * Invalidates the cached hub parameters, forcing recomputation on the next {@link
   * #getHubTargetParameters()} call. Should be called once per loop cycle.
   */
  public void clearHubLaunchingParameters() {
    latestHubTargetParameters = null;
  }

  /**
   * Invalidates the cached passing parameters, forcing recomputation on the next {@link
   * #getPassingTargetParameters()} call. Should be called once per loop cycle.
   */
  public void clearPassingLaunchingParameters() {
    latestPassingTargetParameters = null;
  }

  public static Time getMinHubFuelTimeOfFlight() {
    return (Time) FUEL_HUB_TIME_OF_FLIGHT_MAP.get(MINIMUM_HUB_SHOOTING_DISTANCE);
  }

  public static Time getMaxHubFuelTimeOfFlight() {
    return (Time) FUEL_HUB_TIME_OF_FLIGHT_MAP.get(MAXIMUM_HUB_SHOOTING_DISTANCE);
  }

  public static Time getMinPassingFuelTimeOfFlight() {
    return (Time) FUEL_PASSING_TIME_OF_FLIGHT_MAP.get(MINIMUM_PASSING_SHOOTING_DISTANCE);
  }

  public static Time getMaxPassingFuelTimeOfFlight() {
    return (Time) FUEL_PASSING_TIME_OF_FLIGHT_MAP.get(MAXIMUM_PASSING_SHOOTING_DISTANCE);
  }
}