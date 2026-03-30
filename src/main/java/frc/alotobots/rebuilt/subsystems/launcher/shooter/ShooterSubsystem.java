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
package frc.alotobots.rebuilt.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.SHOOTER_LIMITS_ENABLED;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.SHOOTER_MAX_OPEN_LOOP_PERCENTAGE;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.SHOOTER_MAX_VELOCITY;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Thresholds.SHOOTER_AT_TARGET_VELOCITY_SPEED_THRESHOLD;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Thresholds.SHOOTER_AT_TARGET_VELOCITY_TIME_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIO;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Debouncer atTargetVelocityDebounce =
      new Debouncer(SHOOTER_AT_TARGET_VELOCITY_TIME_THRESHOLD.in(Seconds));

  @AutoLogOutput(key = "Launcher/Shooter/TargetVelocity")
  private AngularVelocity targetVelocity = RadiansPerSecond.zero();

  private SysIdRoutine sysIdRoutine;

  public ShooterSubsystem(ShooterIO io) {
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(3).per(Second),
                Volts.of(6),
                null,
                (state) -> Logger.recordOutput("SysId/Shooter/State", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> this.runAtVoltage(voltage), null, this));
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher/Shooter", inputs);
  }

  /**
   * Controls the shooter to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained between
   *     -MAX_VELOCITY and MAX_VELOCITY
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -SHOOTER_MAX_VELOCITY.in(RadiansPerSecond),
                SHOOTER_MAX_VELOCITY.in(RadiansPerSecond)));
    targetVelocity = SHOOTER_LIMITS_ENABLED ? adjustedVelocity : velocity;
    io.setShooterVelocity(targetVelocity);
    Logger.recordOutput("Launcher/Shooter/ControlType", ShooterIO.PIDSlots.DEFAULT_VELOCITY);
  }

  public void runShooterPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(
            percentOutput, -SHOOTER_MAX_OPEN_LOOP_PERCENTAGE, SHOOTER_MAX_OPEN_LOOP_PERCENTAGE);
    io.setShooterOpenLoop(SHOOTER_LIMITS_ENABLED ? adjustedOutput : percentOutput);
  }

  private void runAtVoltage(Voltage voltageOut) {
    io.setShooterVoltage(voltageOut);
  }

  /**
   * Checks if the shooter is stably at its target velocity for a minimum duration.
   *
   * @return true if the shooter has maintained its target velocity within tolerance
   */
  public boolean isAtTargetVelocity() {
    // Check if current velocity is within threshold of target
    boolean inSetPointThreshold =
        targetVelocity.minus(inputs.shooterMotorLeftVelocity).abs(RadiansPerSecond)
            < SHOOTER_AT_TARGET_VELOCITY_SPEED_THRESHOLD.in(RadiansPerSecond);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetVelocityDebounce.calculate(inSetPointThreshold);
  }

  public AngularVelocity getCurrentVelocity() {
    // Return only the left as they are physically linked
    return inputs.shooterMotorLeftVelocity;
  }

  public void stop() {
    io.stop();
    targetVelocity = RadiansPerSecond.zero();
  }

  public Command sysIdFwdDynamic() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdRvsDynamic() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFwdQuasistatic() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdRvsQuasiStatic() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }
}
