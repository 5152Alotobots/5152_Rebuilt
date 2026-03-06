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
package frc.alotobots.rebuilt.subsystems.intake.extendo;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Limits.*;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Thresholds.AT_TARGET_EXTENSION_POSITION_THRESHOLD;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Thresholds.AT_TARGET_EXTENSION_TIME_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIO;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IntakeExtendoSubsystem extends SubsystemBase {

  private IntakeExtendoIO io;
  private IntakeExtendoIOInputsAutoLogged inputs = new IntakeExtendoIOInputsAutoLogged();

  /**
   * Distance object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Distance targetExtension = Meters.zero();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetExtensionDebounce =
      new Debouncer(AT_TARGET_EXTENSION_TIME_THRESHOLD.in(Seconds));

  /** Debouncer for retracted-state resetting logic */
  private final Debouncer retractedDebounce =
      new Debouncer(AT_TARGET_EXTENSION_TIME_THRESHOLD.in(Seconds));

  public IntakeExtendoSubsystem(IntakeExtendoIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Extendo", inputs);
  }

  /**
   * Controls the extendo to move to a specified extension using closed-loop motion-magic position
   * control.
   *
   * @param extension Target extension, automatically constrained between MIN_HEIGHT and MAX_HEIGHT
   */
  public void runToTargetPosition(Distance extension) {
    Distance adjustedExtension =
        Meters.of(
            MathUtil.clamp(
                extension.in(Meters), MIN_EXTENSION.in(Meters), MAX_EXTENSION.in(Meters)));
    targetExtension = LIMITS_ENABLED ? adjustedExtension : extension;
    io.setIntakeExtendoPosition(targetExtension, IntakeExtendoIO.PIDSlots.POSITION);
    Logger.recordOutput("Intake/Extendo/ControlType", IntakeExtendoIO.PIDSlots.POSITION);
  }

  /**
   * Controls the intake extendo to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runToTargetVelocity(LinearVelocity velocity) {
    LinearVelocity adjustedVelocity = applyVelocityLimitIfNeeded(velocity);
    io.setIntakeExtendoVelocity(LIMITS_ENABLED ? adjustedVelocity : velocity);
    Logger.recordOutput("Intake/Extendo/ControlType", IntakeExtendoIO.PIDSlots.VELOCITY);
  }

  public void runAtPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setIntakeExtendoOpenLoop(adjustedOutput);
    Logger.recordOutput("Intake/Extendo/ControlType", "PERCENT_OUTPUT");
  }

  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current extension of the extendo.
   *
   * @return The current extension as a Distance object
   */
  public Distance getCurrentExtension() {
    return inputs.intakeExtendoDistance;
  }

  /**
   * Checks if the extendo is stably at its target extension for a minimum duration.
   *
   * @return true if the extendo has maintained its target height within tolerance
   */
  public boolean isAtTargetExtension() {
    // Check if current extension is within threshold of target
    boolean inSetPointThreshold =
        targetExtension.minus(inputs.intakeExtendoDistance).abs(Meters)
            < AT_TARGET_EXTENSION_POSITION_THRESHOLD.in(Meters);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetExtensionDebounce.calculate(inSetPointThreshold);
  }

  /**
   * Checks if the extendo is within the velocity limit distance from the top or bottom limits.
   *
   * @return true if the extendo is within the velocity limit distance, false otherwise
   */
  private boolean isVelocityLimitNeeded() {
    if ((inputs.intakeExtendoDistance.minus(MIN_EXTENSION)).abs(Meters)
        < DISTANCE_FROM_LIMIT.in(Meters)) {
      Logger.recordOutput("Intake/Extendo/LimitReason", "NEAR_MIN_EXTENSION_LIMIT");
      return true;
    }
    if ((inputs.intakeExtendoDistance.minus(MAX_EXTENSION)).abs(Meters)
        < DISTANCE_FROM_LIMIT.in(Meters)) {
      Logger.recordOutput("Intake/Extendo/LimitReason", "NEAR_MAX_EXTENSION_LIMIT");
      return true;
    }

    return false;
  }

  /**
   * Applies the velocity limit if the extendo is within the velocity limit distance from the in or
   * out limits.
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
      return MetersPerSecond.of(
          MathUtil.clamp(
              velocity.in(MetersPerSecond),
              -MAX_OPERATOR_VELOCITY.in(MetersPerSecond),
              MAX_OPERATOR_VELOCITY.in(MetersPerSecond)));
    }
  }
}
