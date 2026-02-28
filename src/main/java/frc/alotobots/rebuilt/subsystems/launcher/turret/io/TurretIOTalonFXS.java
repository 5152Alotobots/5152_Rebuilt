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
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;
import static frc.alotobots.Constants.DIO.TURRET_RESET_LIMIT_SWITCH_CHANNEL;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import frc.alotobots.util.PhoenixUtil;

public class TurretIOTalonFXS implements TurretIO {
  protected final TalonFXS turretMotor;
  private final DigitalInput resetLimitSwitch = new DigitalInput(TURRET_RESET_LIMIT_SWITCH_CHANNEL);

  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  private final Debouncer resetLimitDebouncer = new Debouncer(0.5);
  private final Debouncer turretMotorConnectedDebouncer = new Debouncer(0.1);

  private StatusSignal<Angle> turretMotorPosition;
  private StatusSignal<AngularVelocity> turretMotorVelocity;
  private StatusSignal<AngularAcceleration> turretMotorAcceleration;
  private StatusSignal<Voltage> turretMotorVoltage;
  private StatusSignal<Current> turretMotorCurrent;
  private StatusSignal<Integer> currentPidSlot;

  public TurretIOTalonFXS() {
    turretMotor = new TalonFXS(Constants.CanId.TURRET_CAN_ID, RIO_CAN_BUS);

    var turretMotorConfig = new TalonFXSConfiguration();

    // PID configuration for position mode (Slot 0)
    turretMotorConfig.Slot0.kP =
        TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KP;
    turretMotorConfig.Slot0.kD =
        TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KD;
    turretMotorConfig.Slot0.kS =
        TurretTalonFXSConstants.PIDConstants.PositionPIDConstants.TURRET_POSITION_KS;

    // PID configuration for velocity mode (Slot 1)
    turretMotorConfig.Slot1.kP =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KP;
    turretMotorConfig.Slot1.kI =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KI;
    turretMotorConfig.Slot1.kD =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KD;
    turretMotorConfig.Slot1.kG =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KG;
    turretMotorConfig.Slot1.kS =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KS;
    turretMotorConfig.Slot1.kV =
        TurretTalonFXSConstants.PIDConstants.VelocityPIDConstants.TURRET_VELOCITY_KV;

    turretMotorConfig.MotorOutput.NeutralMode =
        TurretTalonFXSConstants.TURRET_MECHANISM_NEUTRAL_MODE;
    turretMotorConfig.MotorOutput.Inverted = TurretTalonFXSConstants.TURRET_MOTOR_DIRECTION;
    turretMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.Commutation;
    turretMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    turretMotorConfig.ExternalFeedback.SensorToMechanismRatio =
        TurretTalonFXSConstants.TURRET_SENSOR_TO_MECHANISM_RATIO;

    turretMotorConfig.CurrentLimits.StatorCurrentLimit =
        TurretTalonFXSConstants.MotorSafetyLimits.TURRET_STATOR_AMP_LIMIT.in(Amps);
    turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretMotorConfig, 0.25));

    turretMotorPosition = turretMotor.getPosition();
    turretMotorVelocity = turretMotor.getVelocity();
    turretMotorAcceleration = turretMotor.getAcceleration();
    turretMotorVoltage = turretMotor.getMotorVoltage();
    turretMotorCurrent = turretMotor.getStatorCurrent();
    currentPidSlot = turretMotor.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        turretMotorPosition,
        turretMotorVelocity,
        turretMotorAcceleration,
        turretMotorVoltage,
        turretMotorCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(turretMotor);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var turretSignals =
        BaseStatusSignal.refreshAll(
            turretMotorPosition,
            turretMotorVelocity,
            turretMotorAcceleration,
            turretMotorVoltage,
            turretMotorCurrent,
            currentPidSlot);

    inputs.turretMotorConnected = turretMotorConnectedDebouncer.calculate(turretSignals.isOK());

    inputs.turretAngle = turretMotorPosition.getValue();

    inputs.turretMotorVelocity = turretMotorVelocity.getValue();

    inputs.turretMotorAcceleration = turretMotorAcceleration.getValue();

    inputs.turretMotorVolts = turretMotorVoltage.getValue();
    inputs.turretMotorCurrent = turretMotorCurrent.getValue();

    inputs.turretMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          case 1 -> PIDSlots.VELOCITY;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };
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
    if (pidSlot == PIDSlots.OPEN_LOOP)
      throw new IllegalArgumentException(
          "PIDSlots value OPEN_LOOP cannot be used in a closed loop control mode");

    turretMotor.setControl(positionControl.withPosition(position).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setTurretVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    if (pidSlot == PIDSlots.OPEN_LOOP)
      throw new IllegalArgumentException(
          "PIDSlots value OPEN_LOOP cannot be used in a closed loop control mode");

    turretMotor.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setTurretVelocity(AngularVelocity velocity) {
    setTurretVelocity(velocity, PIDSlots.VELOCITY);
  }

  @Override
  public void setTurretVoltage(Voltage voltage) {
    turretMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void setTurretOpenLoop(double percentOutput) {

    turretMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    turretMotor.stopMotor();
  }
}
