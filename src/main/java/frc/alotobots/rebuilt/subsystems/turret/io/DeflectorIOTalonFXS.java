/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.turret.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.turret.constants.TurretTalonFXSConstants;
import frc.alotobots.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class DeflectorIOTalonFXS implements DeflectorIO {
  private final TalonFXS deflectorMotor;
  private final CANBus canBus = new CANBus("rio");
  private final PositionVoltage positionControl = new PositionVoltage(0);

  private final DigitalInput backLimitSwitch;
  private final Debouncer backLimitDebouncer;
  private final Debouncer deflectorMotorConnectedDebouncer;

  private StatusSignal<Angle> deflectorMotorPosition;
  private StatusSignal<AngularVelocity> deflectorMotorVelocity;
  private StatusSignal<AngularAcceleration> deflectorMotorAcceleration;
  private StatusSignal<Voltage> deflectorMotorVoltage;
  private StatusSignal<Current> deflectorMotorCurrent;
  private StatusSignal<Integer> deflectorMotorPidSlot;
  private StatusSignal<ControlModeValue> deflectorMotorControlMode;

  public DeflectorIOTalonFXS() {
    deflectorMotor = new TalonFXS(Constants.CanId.TURRET_CAN_ID, canBus);
    deflectorMotorConnectedDebouncer = new Debouncer(0.5);
    backLimitDebouncer = new Debouncer(0.1);
    backLimitSwitch = new DigitalInput(0);

    var deflectorMotorConfig = new TalonFXSConfiguration();
    deflectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deflectorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deflectorMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.Commutation;
    deflectorMotorConfig.ExternalFeedback.SensorToMechanismRatio =
        TurretTalonFXSConstants.SENSOR_TO_MECHANISM_RATIO;
    deflectorMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    deflectorMotorConfig.Slot0.kP = TurretTalonFXSConstants.POSITION_P_GAIN;
    deflectorMotorConfig.Slot0.kI = TurretTalonFXSConstants.POSITION_I_GAIN;
    deflectorMotorConfig.Slot0.kD = TurretTalonFXSConstants.POSITION_D_GAIN;

    PhoenixUtil.tryUntilOk(
        5, () -> deflectorMotor.getConfigurator().apply(deflectorMotorConfig, 0.25));

    deflectorMotorPosition = deflectorMotor.getPosition();
    deflectorMotorVelocity = deflectorMotor.getVelocity();
    deflectorMotorAcceleration = deflectorMotor.getAcceleration();
    deflectorMotorVoltage = deflectorMotor.getMotorVoltage();
    deflectorMotorCurrent = deflectorMotor.getStatorCurrent();
    deflectorMotorPidSlot = deflectorMotor.getClosedLoopSlot();
    deflectorMotorControlMode = deflectorMotor.getControlMode();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        deflectorMotorPosition,
        deflectorMotorVelocity,
        deflectorMotorAcceleration,
        deflectorMotorVoltage,
        deflectorMotorCurrent,
        deflectorMotorPidSlot,
        deflectorMotorControlMode);

    ParentDevice.optimizeBusUtilizationForAll(deflectorMotor);
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    var motorSignals =
        BaseStatusSignal.refreshAll(
            deflectorMotorPosition,
            deflectorMotorVelocity,
            deflectorMotorAcceleration,
            deflectorMotorVoltage,
            deflectorMotorCurrent,
            deflectorMotorControlMode);

    inputs.deflectorMotorPidSlot =
        switch (deflectorMotorPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + deflectorMotorPidSlot.getValue());
        };

    inputs.deflectorMotorControlMode = deflectorMotorControlMode.getValue();
    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(motorSignals.isOK());
    inputs.deflectorMotorVelocity = deflectorMotorVelocity.getValue();
    inputs.deflectorMotorAcceleration = deflectorMotorAcceleration.getValue();
    inputs.deflectorMotorVolts = deflectorMotorVoltage.getValue();
    inputs.deflectorMotorCurrent = deflectorMotorCurrent.getValue();
    inputs.deflectorMotorPosition = deflectorMotorPosition.getValue();
  }

  @Override
  public void setDeflectorPosition(Angle position) {
    setDeflectorPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  /**
   * * Commands the deflector to move to a specified position using closed-loop control.
   *
   * @param position The target position for the deflector
   * @param pidSlot The PID slot to use for the control
   * @throws IllegalArgumentException if position or pidSlot is null, or if pidSlot is invalid
   */
  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    if (position == null) {
      throw new IllegalArgumentException("Position cannot be null");
    }

    if (pidSlot == null) {
      throw new IllegalArgumentException("PID Slot cannot be null");
    }

    if (pidSlot != PIDSlots.DEFAULT_POSITION) {
      throw new IllegalArgumentException(
          "Invalid PID Slot for deflector, Got " + pidSlot.toString());
    }

    Logger.recordOutput("deflector/setpoint", position);

    deflectorMotor.setControl(
        positionControl.withPosition(position.in(Rotations)).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setDeflectorOpenLoop(double percentOutput) {
    Logger.recordOutput("deflector/openLoopPercentOut", percentOutput);

    deflectorMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    deflectorMotor.stopMotor();
  }
}
