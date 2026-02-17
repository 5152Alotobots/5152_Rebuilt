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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import frc.alotobots.util.PhoenixUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TurretIOSim implements TurretIO {
  private final DCMotor turretMotorSim = DCMotor.getMinion(1);
  private final SingleJointedArmSim turretSim =
      new SingleJointedArmSim(
          turretMotorSim,
          TurretTalonFXSConstants.SENSOR_TO_MECHANISM_RATIO,
          TurretTalonFXSConstants.MOMENT_OF_INERTIA,
          .33,
          TurretTalonFXSConstants.MIN_ANGLE.in(Radian),
          TurretTalonFXSConstants.MAX_ANGLE.in(Radian),
          false,
          0.0);

  @AutoLogOutput private double currentOutput = 0.0;
  @AutoLogOutput private double appliedVolts = 0.0;
  private boolean currentControl = false;
  private PIDSlots currentPidSlot = PIDSlots.DEFAULT_POSITION;
  private ControlModeValue turretMotorControlMode = ControlModeValue.DisabledOutput;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (currentControl) {
      appliedVolts = turretMotorSim.getVoltage(currentOutput, turretSim.getVelocityRadPerSec());
    }

    // Update sim state
    turretSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    turretSim.update(0.02);

    inputs.turretMotorPidSlot =
        switch (currentPidSlot.ordinal()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot);
        };

    inputs.ccwLimit = turretSim.hasHitUpperLimit();
    inputs.cwLimit = turretSim.hasHitLowerLimit();
    inputs.turretMotorControlMode = turretMotorControlMode;
    inputs.turretMotorConnected = true;
    inputs.turretMotorVelocity = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());
    inputs.turretMotorVolts = Volts.of(appliedVolts);
    inputs.turretMotorCurrent = Amps.of(turretSim.getCurrentDrawAmps());
    inputs.turretMotorPosition = Radian.of(turretSim.getAngleRads());
  }

  @Override
  public void setTurretPosition(Angle position) {
    setTurretPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  /**
   * * Commands the turret to move to a specified position using closed-loop control.
   *
   * @param position The target position for the turret
   * @param pidSlot The PID slot to use for the control
   * @throws IllegalArgumentException if position or pidSlot is null, or if pidSlot is invalid
   */
  @Override
  public void setTurretPosition(Angle position, PIDSlots pidSlot) {
    if (position == null) {
      throw new IllegalArgumentException("Position cannot be null");
    }

    if (pidSlot == null) {
      throw new IllegalArgumentException("PID Slot cannot be null");
    }

    if (pidSlot != PIDSlots.DEFAULT_POSITION) {
      throw new IllegalArgumentException("Invalid PID Slot for Turret, Got " + pidSlot.toString());
    }

    Logger.recordOutput("Turret/setpoint", position.in(Rotations));

    currentPidSlot = pidSlot;
    currentControl = false;
    turretMotorControlMode = ControlModeValue.PositionVoltage;
    double error = Math.abs(position.minus(Radian.of(turretSim.getAngleRads())).in(Radians));
    appliedVolts =
        PhoenixUtil.calculateVoltageForPositionControl(
            position.in(Rotations),
            turretSim.getAngleRads() / (2 * Math.PI),
            RadiansPerSecond.of(turretSim.getVelocityRadPerSec()).in(RotationsPerSecond),
            TurretTalonFXSConstants.POSITION_P_GAIN,
            TurretTalonFXSConstants.POSITION_D_GAIN);
    error = Math.abs(position.minus(Radian.of(turretSim.getAngleRads())).in(Radians));
  }

  @Override
  public void setTurretOpenLoop(double percentOutput) {
    Logger.recordOutput("Turret/openLoopPercentOut", percentOutput);

    currentControl = true;
    turretMotorControlMode = ControlModeValue.DutyCycleOut;
    currentOutput = percentOutput * turretMotorSim.stallTorqueNewtonMeters;
  }

  @Override
  public void setTurretVoltageOut(Voltage voltageOutput) {
    Logger.recordOutput("Turret/voltageOutput", voltageOutput);

    currentControl = true;
    turretMotorControlMode = ControlModeValue.VoltageOut;
    currentOutput = voltageOutput.in(Volts) * turretMotorSim.stallTorqueNewtonMeters / 12.0;
  }

  @Override
  public void stop() {
    currentControl = true;
    turretMotorControlMode = ControlModeValue.DisabledOutput;
    currentOutput = 0.0;
  }
}
