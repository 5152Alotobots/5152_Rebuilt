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

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.*;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Thresholds.AT_TARGET_ANGLE_POSITION_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIO;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  /** Hardware abstraction for the turret */
  private final TurretIO io;

  /** Latest inputs from the turret hardware */
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  @AutoLogOutput(key = "Launcher/Turret/TargetAngle")
  private Angle targetAngle = Degrees.zero();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(TurretConstants.Thresholds.AT_TARGET_ANGLE_TIME_THRESHOLD.in(Seconds));

  /**
   * Creates a new TurretSubsystem.
   *
   * @param io The hardware abstraction interface for the turret
   */
  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.processInputs("Launcher/Turret", inputs);
  }

  /**
   * Commands the turret to move to a target angle using closed-loop control.
   *
   * @param angle The target angle for the turret
   */
  public void runToTargetAngle(Angle angle) {
    Angle adjustedAngle =
        Radians.of(MathUtil.clamp(angle.in(Radians), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians)));

    targetAngle = LIMITS_ENABLED ? adjustedAngle : angle;
    io.setTurretPosition(targetAngle);
    Logger.recordOutput("Launcher/Turret/ControlType", TurretIO.PIDSlots.DEFAULT_POSITION);
  }

  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -MAX_VELOCITY.in(RadiansPerSecond),
                MAX_VELOCITY.in(RadiansPerSecond)));
    io.setTurretVelocity(LIMITS_ENABLED ? adjustedVelocity : velocity);
    Logger.recordOutput("Launcher/Turret/ControlType", TurretIO.PIDSlots.VELOCITY);
  }

  /**
   * Runs the turret using direct percent output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    // Clamp percent output
    double adjustedSpeed =
        MathUtil.clamp(
            percentOutput,
            -TurretConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE,
            TurretConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE);

    io.setTurretOpenLoop(adjustedSpeed);
    Logger.recordOutput("Launcher/Turret/ControlType", TurretIO.PIDSlots.OPEN_LOOP);
  }

  /** Stops all turret movement. */
  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current angle of the turret.
   *
   * @return The current angle as an Angle object
   */
  public Angle getCurrentAngle() {
    return inputs.turretAngle;
  }

  /**
   * Checks if the turret is stably at its target angle for a minimum duration.
   *
   * @return true if the turret has maintained its target angle within tolerance
   */
  public boolean isAtTargetAngle() {
    // Check if current angle is within threshold of target
    boolean inSetPointThreshold =
        targetAngle.minus(inputs.turretAngle).abs(Radians)
            < AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Radians);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
