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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import frc.alotobots.rebuilt.subsystems.turret.constants.TurretTalonFXSConstants;
import frc.alotobots.util.PhoenixUtil;

import org.littletonrobotics.junction.Logger;



public class TurretIOTalonFXS implements TurretIO {
  private final TalonFXS turretMotor;
  private final CANBus canBus = new CANBus("rio");
  private final DigitalInput cwLimitSwitch;
  private final DigitalInput ccwLimitSwitch;
  private final PositionVoltage positionControl = new PositionVoltage(0);

  private final Debouncer cwLimitDebouncer;
  private final Debouncer ccwLimitDebouncer;
  private final Debouncer turretMotorConnectedDebouncer;

  private StatusSignal<Angle> motorPosition;
  private StatusSignal<AngularVelocity> motorVelocity;
  private StatusSignal<AngularAcceleration> motorAcceleration;
  private StatusSignal<Voltage> motorAppliedVoltage;
  private StatusSignal<Current> motorAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;

  public TurretIOTalonFXS() {
    turretMotor = new TalonFXS(Constants.CanId.TURRET_CAN_ID, canBus);
    ccwLimitDebouncer = new Debouncer(0.1);
    cwLimitDebouncer = new Debouncer(0.1);
    turretMotorConnectedDebouncer = new Debouncer(0.5);
    ccwLimitSwitch = new DigitalInput(0);
    cwLimitSwitch = new DigitalInput(1);

    var motorMotorConfig = new TalonFXSConfiguration();
    motorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature;
    motorMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    motorMotorConfig.Slot0.kP = TurretTalonFXSConstants.POSITION_P_GAIN;
    motorMotorConfig.Slot0.kI = TurretTalonFXSConstants.POSITION_I_GAIN;
    motorMotorConfig.Slot0.kD = TurretTalonFXSConstants.POSITION_D_GAIN;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorMotorConfig, 0.25));

    motorPosition = turretMotor.getPosition();
    motorVelocity = turretMotor.getVelocity();
    motorAcceleration = turretMotor.getAcceleration();
    motorAppliedVoltage = turretMotor.getMotorVoltage();
    motorAppliedCurrent = turretMotor.getStatorCurrent();
    currentPidSlot = turretMotor.getClosedLoopSlot();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPosition,
        motorVelocity,
        motorAcceleration,
        motorAppliedVoltage,
        motorAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(turretMotor);

  }
 

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var motorSignals =
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            motorAcceleration,
            motorAppliedVoltage,
            motorAppliedCurrent);

    inputs.pidSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot.getValue());
        };

    inputs.motorConnected = turretMotorConnectedDebouncer.calculate(motorSignals.isOK());
    inputs.rotationVelocity = motorVelocity.getValue();
    inputs.rotationAcceleration = motorAcceleration.getValue();
    inputs.motorAppliedVolts = motorAppliedVoltage.getValue();
    inputs.motorCurrent = motorAppliedCurrent.getValue();
    inputs.mechanismAngle = motorPosition.getValue();
  }

  @Override
  public void setTurretPosition(Angle position) {
    setTurretPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  @Override
  public void setTurretPosition(Angle position, PIDSlots pidSlot) {
    Logger.recordOutput("turretIO/setpoint", position);
    turretMotor.setControl(positionControl.withPosition(position.in(Rotations)).withSlot(pidSlot.ordinal())); 
  }

  @Override
  public void setTurretOpenLoop(double percentOutput) {
    // Set SparkMax to open-loop control with given percentage output
    turretMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    // Stop SparkMax motor
    turretMotor.stopMotor();
  }
}
