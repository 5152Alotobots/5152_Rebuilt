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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.io;

import static edu.wpi.first.units.Units.Rotations;
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.alotobots.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class DeflectorIOTalonFXS implements DeflectorIO {
  private final TalonFXS deflectorMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  private final DigitalInput backLimitSwitch;
  private final Debouncer backLimitDebouncer;
  private final Debouncer deflectorMotorConnectedDebouncer;

  private StatusSignal<Angle> deflectorMotorPosition;
  private StatusSignal<AngularVelocity> deflectorMotorVelocity;
  private StatusSignal<AngularAcceleration> deflectorMotorAcceleration;
  private StatusSignal<Voltage> deflectorMotorVoltage;
  private StatusSignal<Current> deflectorMotorCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private StatusSignal<ControlModeValue> deflectorMotorControlMode;

  public DeflectorIOTalonFXS() {
    deflectorMotor = new TalonFXS(Constants.CanId.DEFLECTOR_MOTOR_CAN_ID, RIO_CAN_BUS);
    deflectorMotorConnectedDebouncer = new Debouncer(0.5);
    backLimitDebouncer = new Debouncer(0.1);
    backLimitSwitch = new DigitalInput(0);

    var deflectorMotorConfig = new TalonFXSConfiguration();
    deflectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deflectorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deflectorMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.Commutation;
    deflectorMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // PID Slots config removed here. This is currently not in use. IF IT IS EVER USED THIS MUST BE
    // CONFIGURED

    PhoenixUtil.tryUntilOk(
        5, () -> deflectorMotor.getConfigurator().apply(deflectorMotorConfig, 0.25));

    deflectorMotorPosition = deflectorMotor.getPosition();
    deflectorMotorVelocity = deflectorMotor.getVelocity();
    deflectorMotorAcceleration = deflectorMotor.getAcceleration();
    deflectorMotorVoltage = deflectorMotor.getMotorVoltage();
    deflectorMotorCurrent = deflectorMotor.getStatorCurrent();
    currentPidSlot = deflectorMotor.getClosedLoopSlot();
    deflectorMotorControlMode = deflectorMotor.getControlMode();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        deflectorMotorPosition,
        deflectorMotorVelocity,
        deflectorMotorAcceleration,
        deflectorMotorVoltage,
        deflectorMotorCurrent,
        currentPidSlot,
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
            currentPidSlot,
            deflectorMotorControlMode);

    inputs.deflectorMotorPidSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          case 1 -> PIDSlots.VELOCITY;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };

    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(motorSignals.isOK());
    inputs.deflectorMotorVelocity = deflectorMotorVelocity.getValue();
    inputs.deflectorMotorAcceleration = deflectorMotorAcceleration.getValue();
    inputs.deflectorMotorVolts = deflectorMotorVoltage.getValue();
    inputs.deflectorMotorCurrent = deflectorMotorCurrent.getValue();
    inputs.deflectorMotorAngle = deflectorMotorPosition.getValue();
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

    deflectorMotor.setControl(
        positionControl.withPosition(position.in(Rotations)).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    deflectorMotor.setControl(velocityControl.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity) {
    deflectorMotor.setControl(
        velocityControl.withVelocity(velocity).withSlot(PIDSlots.VELOCITY.ordinal()));
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
