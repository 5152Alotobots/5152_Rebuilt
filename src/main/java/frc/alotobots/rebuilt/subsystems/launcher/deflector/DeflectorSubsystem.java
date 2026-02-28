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
package frc.alotobots.rebuilt.subsystems.launcher.deflector;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.*;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Thresholds.DEFLECTOR_AT_TARGET_ANGLE_POSITION_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DeflectorSubsystem extends SubsystemBase {
  /** Hardware abstraction for the deflector */
  private final DeflectorIO io;

  /** Latest inputs from the deflector hardware */
  private final DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  /**
   * Angle object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  @AutoLogOutput(key = "Launcher/Deflector/TargetAngle")
  private Angle targetAngle = Degrees.zero();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(
          DeflectorConstants.Thresholds.DEFLECTOR_AT_TARGET_ANGLE_TIME_THRESHOLD.in(Seconds));

  /**
   * Creates a new DeflectorSubsystem.
   *
   * @param io The hardware abstraction interface for the deflector
   */
  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.processInputs("Launcher/Deflector", inputs);
  }

  /**
   * Commands the deflector to move to a target angle using closed-loop control.
   *
   * @param angle The target angle for the deflector
   */
  public void runToTargetAngle(Angle angle) {
    Angle adjustedAngle =
        Radians.of(
            MathUtil.clamp(
                angle.in(Radians),
                DEFLECTOR_MIN_ANGLE.in(Radians),
                DEFLECTOR_MAX_ANGLE.in(Radians)));
    targetAngle = DEFLECTOR_LIMITS_ENABLED ? adjustedAngle : angle;
    io.setDeflectorPosition(targetAngle, DeflectorIO.PIDSlots.DEFAULT_POSITION);

    Logger.recordOutput("Launcher/Deflector/ControlType", DeflectorIO.PIDSlots.DEFAULT_POSITION);
  }

  /**
   * Controls the deflector to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained between
   *     -MAX_VELOCITY and MAX_VELOCITY
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -DEFLECTOR_MAX_VELOCITY.in(RadiansPerSecond),
                DEFLECTOR_MAX_VELOCITY.in(RadiansPerSecond)));
    io.setDeflectorVelocity(DEFLECTOR_LIMITS_ENABLED ? adjustedVelocity : velocity);
    Logger.recordOutput("Launcher/Deflector/ControlType", DeflectorIO.PIDSlots.VELOCITY);
  }

  /**
   * Runs the deflector using direct percent output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    // Clamp percent output
    double adjustedSpeed =
        MathUtil.clamp(
            percentOutput,
            -DeflectorConstants.Limits.DEFLECTOR_MAX_OPEN_LOOP_PERCENTAGE,
            DeflectorConstants.Limits.DEFLECTOR_MAX_OPEN_LOOP_PERCENTAGE);

    io.setDeflectorOpenLoop(DEFLECTOR_LIMITS_ENABLED ? adjustedSpeed : percentOutput);
    Logger.recordOutput("Launcher/Deflector/ControlType", DeflectorIO.PIDSlots.OPEN_LOOP);
  }

  /** Stops all deflector movement. */
  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current angle of the deflector.
   *
   * @return The current angle as an Angle object
   */
  public Angle getCurrentAngle() {
    return inputs.deflectorAngle;
  }

  /**
   * Checks if the deflector is stably at its target angle for a minimum duration.
   *
   * @return true if the deflector has maintained its target angle within tolerance
   */
  public boolean isAtTargetAngle() {
    // Check if current angle is within threshold of target
    boolean inSetPointThreshold =
        targetAngle.minus(inputs.deflectorAngle).abs(Radians)
            < DEFLECTOR_AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Radians);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
