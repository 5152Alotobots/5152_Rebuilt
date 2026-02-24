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
package frc.alotobots.rebuilt.subsystems.launcher.turret.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation implementation of the Turret IO layer.
 *
 * <p>This class simulates the physical behavior of the turret using WPILib's {@link
 * SingleJointedArmSim}. It handles closed-loop control internally to mimic the behavior of a smart
 * motor controller in simulation.
 */
public class TurretIOSim implements TurretIO {

  // --- Simulation Constants & Objects ---

  private final DCMotor turretMotorSim = DCMotor.getMinion(1);

  /**
   * The physics simulation for the turret.
   *
   * <p>SingleJointedArmSim is used because it supports gravity (if needed) and limits, though a
   * turret is often just a flywheel on its side.
   */
  private final SingleJointedArmSim turretSim =
      new SingleJointedArmSim(
          turretMotorSim,
          TurretTalonFXSConstants.TURRET_SENSOR_TO_MECHANISM_RATIO,
          TurretTalonFXSConstants.MOMENT_OF_INERTIA,
          0.33, // Arm length (approximate radius of turret)
          TurretConstants.Limits.TURRET_MIN_ANGLE.in(Radian),
          TurretConstants.Limits.TURRET_MAX_ANGLE.in(Radian),
          false, // Simulate gravity (false for a standard upright turret)
          0.0 // Starting angle
          );

  /** Internal PID controller to mimic hardware PID behavior. */
  private final PIDController pid =
      new PIDController(
          TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KP,
          TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KI,
          TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KD);

  // --- State Variables ---

  @AutoLogOutput private double currentOutput = 0.0;
  @AutoLogOutput private double appliedVolts = 0.0;

  private boolean currentControl = false;
  private PIDSlots currentPidSlot = PIDSlots.DEFAULT_POSITION;
  private ControlModeValue turretMotorControlMode = ControlModeValue.DisabledOutput;
  private double targetPositionRads = 0.0;

  /**
   * Updates the inputs structure with the latest simulation state.
   *
   * <p>This method performs the following logic:
   *
   * <ul>
   *   <li>Calculates the closed-loop output if in position mode.
   *   <li>Updates the physics simulation with the applied voltage.
   *   <li>Populates the {@link TurretIOInputs} object with simulated sensor data.
   * </ul>
   *
   * @param inputs The input object to update.
   */
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // 1. Calculate Control Output
    if (currentControl && turretMotorControlMode == ControlModeValue.PositionVoltage) {
      // Calculate PID output based on current error
      double pidOutput = pid.calculate(turretSim.getAngleRads(), targetPositionRads);
      appliedVolts = pidOutput;
    } else if (currentControl) {
      // In open loop/voltage modes, appliedVolts is derived from the requested output
      // Note: getVoltage expects speed in Rad/s to calculate Back-EMF correctly
      appliedVolts = turretMotorSim.getVoltage(currentOutput, turretSim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    // 2. Update Physics
    turretSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    turretSim.update(0.02); // Standard loop time of 20ms

    // 3. Update Inputs
    inputs.turretMotorPIDSlot =
        switch (currentPidSlot.ordinal()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          default -> throw new IllegalStateException(
              "Invalid PID Slot in Turret Sim: " + currentPidSlot);
        };

    inputs.resetLimit = turretSim.hasHitUpperLimit();
    inputs.turretMotorConnected = true;

    // Sensor Data
    inputs.turretMotorVelocity = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());
    inputs.turretMotorVolts = Volts.of(appliedVolts);
    inputs.turretMotorCurrent = Amps.of(turretSim.getCurrentDrawAmps());
    inputs.turretAngle = Radian.of(turretSim.getAngleRads());
  }

  /**
   * Sets the turret position using the default PID slot.
   *
   * @param position The target angle.
   */
  @Override
  public void setTurretPosition(Angle position) {
    setTurretPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  /**
   * Commands the turret to move to a specified position using closed-loop control.
   *
   * @param position The target position for the turret.
   * @param pidSlot The PID slot to use for the control.
   * @throws IllegalArgumentException if position or pidSlot is null.
   */
  @Override
  public void setTurretPosition(Angle position, PIDSlots pidSlot) {
    if (position == null) throw new IllegalArgumentException("Position cannot be null");
    if (pidSlot == null) throw new IllegalArgumentException("PID Slot cannot be null");

    Logger.recordOutput("Turret/setpoint", position.in(Rotations));

    this.targetPositionRads = position.in(Radians);
    this.currentPidSlot = pidSlot;
    this.currentControl = true;
    this.turretMotorControlMode = ControlModeValue.PositionVoltage;
  }

  /**
   * Runs the turret motor at a specified percentage output (Open Loop).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0).
   */
  @Override
  public void setTurretOpenLoop(double percentOutput) {
    Logger.recordOutput("Turret/openLoopPercentOut", percentOutput);

    this.currentControl = true;
    this.turretMotorControlMode = ControlModeValue.DutyCycleOut;
    // Calculate torque required for this percent output for simulation
    this.currentOutput = percentOutput * turretMotorSim.stallTorqueNewtonMeters;
  }

  /** Stops the turret motor and disables control. */
  @Override
  public void stop() {
    this.currentControl = true;
    this.turretMotorControlMode = ControlModeValue.DisabledOutput;
    this.currentOutput = 0.0;
    this.appliedVolts = 0.0;
  }
}
