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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.TURRET_LIMITS_ENABLED;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.TURRET_MAX_ANGLE;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.TURRET_MAX_VELOCITY;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.TURRET_MIN_ANGLE;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Thresholds.TURRET_AT_TARGET_ANGLE_POSITION_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
      new Debouncer(TurretConstants.Thresholds.TURRET_AT_TARGET_ANGLE_TIME_THRESHOLD.in(Seconds));

  private final SysIdRoutine sysIdRoutine;

  /**
   * Creates a new TurretSubsystem.
   *
   * @param io The hardware abstraction interface for the turret
   */
  public TurretSubsystem(TurretIO io) {
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.3).per(Second),
                Volts.of(1.2),
                null,
                (state) -> Logger.recordOutput("SysId/State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runAtVoltage(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

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
        Radians.of(
            MathUtil.clamp(
                angle.in(Radians), TURRET_MIN_ANGLE.in(Radians), TURRET_MAX_ANGLE.in(Radians)));

    targetAngle = TURRET_LIMITS_ENABLED ? adjustedAngle : angle;
    io.setTurretPosition(targetAngle);
    Logger.recordOutput("Launcher/Turret/ControlType", TurretIO.PIDSlots.DEFAULT_POSITION);
  }

  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -TURRET_MAX_VELOCITY.in(RadiansPerSecond),
                TURRET_MAX_VELOCITY.in(RadiansPerSecond)));
    io.setTurretVelocity(TURRET_LIMITS_ENABLED ? adjustedVelocity : velocity);
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
            -TurretConstants.Limits.TURRET_MAX_OPEN_LOOP_PERCENTAGE,
            TurretConstants.Limits.TURRET_MAX_OPEN_LOOP_PERCENTAGE);

    io.setTurretOpenLoop(adjustedSpeed);
    Logger.recordOutput("Launcher/Turret/ControlType", "OPEN_LOOP");
  }

  private void runAtVoltage(Voltage voltage) {
    Voltage adjustedVoltage =
        Volts.of(
            MathUtil.clamp(
                voltage.in(Volts),
                -TurretConstants.Limits.TURRET_MAX_VOLTAGE.in(Volts),
                TurretConstants.Limits.TURRET_MAX_VOLTAGE.in(Volts)));

    io.setTurretVoltage(TurretConstants.Limits.TURRET_LIMITS_ENABLED ? adjustedVoltage : voltage);
    Logger.recordOutput("Launcher/Turret/ControlType", "OPEN_LOOP_VOLTAGE");
  }

  /**
   * @return the SysId forward quasi-static command
   */
  public Command sysIdFwdQuasiStatic() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  /**
   * @return the SysId reverse quasi-static command
   */
  public Command sysIdRevQuasiStatic() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  /**
   * @return the SysId dynamic forward command
   */
  public Command sysIdFwdDynamic() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  /**
   * @return the SysId dynamic reverse command
   */
  public Command sysIdRevDynamic() {
    return sysIdRoutine.dynamic(Direction.kReverse);
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
  @AutoLogOutput
  public boolean isAtTargetAngle() {
    // Check if current angle is within threshold of target
    boolean inSetPointThreshold =
        targetAngle.minus(inputs.turretAngle).abs(Radians)
            < TURRET_AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Radians);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
