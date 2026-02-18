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
package frc.alotobots.rebuilt.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.alotobots.rebuilt.subsystems.turret.constants.TurretConstants;
import frc.alotobots.rebuilt.subsystems.turret.io.TurretIO;
import frc.alotobots.rebuilt.subsystems.turret.io.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  /** Hardware abstraction for the wrist */
  private final TurretIO io;

  /** Latest inputs from the wrist hardware */
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(TurretConstants.AT_TARGET_ANGLE_TIME_THRESHOLD);

  private final SysIdRoutine sysIdRoutine;
  private Angle targetAngle = Degrees.zero();

  /**
   * Creates a new TurretSubsystem.
   *
   * @param io The hardware abstraction interface for the wrist
   * @param elevatorHeightSupplier Supplier function that provides the current elevator height
   */
  public TurretSubsystem(TurretIO io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runAtVoltage(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Turret/TargetAngle", targetAngle.in(Degree));
    Logger.processInputs("Turret", inputs);
  }

  /**
   * Commands the wrist to move to a target angle using closed-loop control.
   *
   * @param angle The target angle for the wrist
   */
  public void runToTargetAngle(Angle angle) {

    var adjustedAngle =
        MathUtil.clamp(angle.in(Radian), Degrees.of(-90).in(Radian), Degrees.of(90).in(Radians));

    try {
      io.setTurretPosition(Radians.of(adjustedAngle), TurretIO.PIDSlots.DEFAULT_POSITION);
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    Logger.recordOutput("Turret/TargetAngle", angle);
    Logger.recordOutput("Turret/AdjustedTargetAngle", targetAngle);
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
            TurretConstants.MIN_OPEN_LOOP_PERCENTAGE,
            TurretConstants.MAX_OPEN_LOOP_PERCENTAGE);

    // Command the wrist with the adjusted output and dynamic limits
    io.setTurretOpenLoop(adjustedSpeed);
  }

  public void runAtVoltage(Voltage voltageOutput) {
    io.setTurretVoltageOut(voltageOutput);
  }

  /** Stops all wrist movement. */
  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current angle of the turret.
   *
   * @return The current angle as an Angle object
   */
  public Angle getCurrentAngle() {
    return inputs.turretMotorPosition;
  }

  public Command sysIDQuasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command sysIDQuasistaticReverse() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command sysIDDynamicForward() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command sysIDDynamicReverse() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }

  /**
   * Checks if the turret is stably at its target angle for a minimum duration.
   *
   * @return true if the turret has maintained its target angle within tolerance
   */
  public boolean isAtTargetAngle() {
    // Check if current angle is within threshold of target

    Angle error = targetAngle.minus(inputs.turretMotorPosition);

    Logger.recordOutput("Turret/error", error);

    boolean inSetPointThreshold =
        error.abs(Degree) < TurretConstants.AT_TARGET_ANGLE_THRESHOLD.in(Degrees);

    Logger.recordOutput("Turret/inSetPointThreshold", inSetPointThreshold);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
