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
package frc.alotobots.rebuilt.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants.Limits.*;
import static frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants.Thresholds.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIO;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private Distance targetPosition = Meters.of(0.0);
  private final Debouncer atTargetDebouncer =
      new Debouncer(AT_TARGET_CLIMB_TIME_THRESHOLD.in(Seconds), Debouncer.DebounceType.kRising);

  /**
   * Creates a new ClimberSubsystem.
   *
   * @param io The IO implementation to use
   */
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/TargetPosition", targetPosition);
    Logger.recordOutput("Climber/AtTarget", isAtTargetExtension());
  }

  /**
   * Sets the target position of the climber. The position is clamped between the min and max
   * extension limits.
   *
   * @param position The target position
   */
  public void runClimberToTargetPosition(Distance position) {
    targetPosition =
        Meters.of(
            MathUtil.clamp(
                position.in(Meters),
                MIN_CLIMB_EXTENSION.in(Meters),
                MAX_CLIMB_EXTENSION.in(Meters)));
    io.setClimberPosition(targetPosition);
  }

  public void setClimberOpenLoop(double percentOut) {
    io.setClimberOpenLoop(percentOut);
  }

  /** Stops the climber motor. */
  public void stop() {
    io.stop();
  }

  /**
   * Returns whether the climber is at the target position within the threshold.
   *
   * @return true if at target, false otherwise
   */
  public boolean isAtTargetExtension() {
    return atTargetDebouncer.calculate(
        Math.abs(inputs.climberDistance.minus(targetPosition).in(Meters))
            < AT_TARGET_CLIMB_POSITION_THRESHOLD.in(Meters));
  }

  /**
   * Checks if a velocity limit is needed based on the current position's proximity to hardware
   * limits. * @return true if the climber is within the velocity limit distance, false otherwise
   */
  private boolean isVelocityLimitNeeded() {
    if ((inputs.climberDistance.minus(MIN_CLIMB_EXTENSION)).abs(Meters)
        < DISTANCE_FROM_LIMIT.in(Meters)) {
      Logger.recordOutput("Climber/LimitReason", "NEAR_MIN_LIMIT");
      return true;
    }
    if ((inputs.climberDistance.minus(MAX_CLIMB_EXTENSION)).abs(Meters)
        < DISTANCE_FROM_LIMIT.in(Meters)) {
      Logger.recordOutput("Climber/LimitReason", "NEAR_MAX_LIMIT");
      return true;
    }

    return false;
  }

  /**
   * Applies the velocity limit if the climber is near the physical end-stops.
   *
   * @param velocity The target velocity
   * @return The adjusted velocity if the limit is needed, otherwise the original velocity
   */
  private LinearVelocity applyVelocityLimitIfNeeded(LinearVelocity velocity) {
    if (isVelocityLimitNeeded()) {
      return MetersPerSecond.of(
          MathUtil.clamp(
              velocity.in(MetersPerSecond),
              -MAX_VELOCITY_NEAR_LIMIT.in(MetersPerSecond),
              MAX_VELOCITY_NEAR_LIMIT.in(MetersPerSecond)));
    } else {
      return velocity;
    }
  }
}
