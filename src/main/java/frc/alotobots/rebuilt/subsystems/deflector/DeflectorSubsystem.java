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
package frc.alotobots.rebuilt.subsystems.deflector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.deflector.constnats.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.deflector.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.deflector.io.DeflectorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class DeflectorSubsystem extends SubsystemBase {
  /** Hardware abstraction for the wrist */
  private final DeflectorIO io;

  /** Latest inputs from the wrist hardware */
  private final DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(DeflectorConstants.AT_TARGET_ANGLE_TIME_THRESHOLD);

  /**
   * Angle object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Angle targetAngle = Degrees.zero();

  /**
   * Creates a new DeflectorSubsystem.
   *
   * @param io The hardware abstraction interface for the wrist
   * @param elevatorHeightSupplier Supplier function that provides the current elevator height
   */
  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Deflector/TargetAngle", targetAngle.in(Degree));
    Logger.processInputs("Deflector", inputs);
  }

  /**
   * Commands the wrist to move to a target angle using closed-loop control.
   *
   * @param angle The target angle for the wrist
   */
  public void runToTargetAngle(Angle angle) {
    io.setDeflectorPosition(angle, DeflectorIO.PIDSlots.DEFAULT_POSITION);

    Logger.recordOutput("Deflector/TargetAngle", angle.in(Degrees));
  }

  /**
   * Runs the wrist using direct percent output (open-loop control). Dynamic limits based on current
   * elevator height are passed to the IO layer.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    // Clamp percent output
    double adjustedSpeed =
        MathUtil.clamp(
            percentOutput,
            DeflectorConstants.MIN_OPEN_LOOP_PERCENTAGE,
            DeflectorConstants.MAX_OPEN_LOOP_PERCENTAGE);

    // Command the wrist with the adjusted output and dynamic limits
    io.setDeflectorOpenLoop(adjustedSpeed);
  }

  /** Stops all wrist movement. */
  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current angle of the deflector.
   *
   * @return The current angle as an Angle object
   */
  public Angle getCurrentAngle() {
    return inputs.deflectorMotorPosition;
  }

  /**
   * Checks if the turret is stably at its target angle for a minimum duration.
   *
   * @return true if the turret has maintained its target angle within tolerance
   */
  /*public boolean isAtTargetAngle() {
      // Check if current angle is within threshold of target

      Angle error = targetAngle.minus(inputs.mechanismAngle);

      Logger.recordOutput("Deflector/error", error);

      boolean inSetPointThreshold =
              error.abs(Degree) < AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Degrees);

      Logger.recordOutput("Deflector/inSetPointThreshold", inSetPointThreshold);

      // Use debouncer to check if we've been at setpoint for the required duration
      return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }*/
}
