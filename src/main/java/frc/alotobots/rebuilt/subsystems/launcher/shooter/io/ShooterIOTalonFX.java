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
package frc.alotobots.rebuilt.subsystems.launcher.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private StatusSignal<Angle> leftPosition;
  private StatusSignal<Angle> rightPosition;
  private StatusSignal<AngularVelocity> leftVelocity;
  private StatusSignal<AngularVelocity> rightVelocity;
  private StatusSignal<AngularAcceleration> leftAcceleration;
  private StatusSignal<AngularAcceleration> rightAcceleration;
  private StatusSignal<Voltage> leftAppliedVoltage;
  private StatusSignal<Voltage> rightAppliedVoltage;
  private StatusSignal<Current> leftAppliedCurrent;
  private StatusSignal<Current> rightAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private Debouncer leftConnectedDebounce = new Debouncer(0.1);
  private Debouncer rightConnectedDebounce = new Debouncer(0.1);

  public ShooterIOTalonFX() {
    motorLeft = new TalonFX(Constants.CanId.SHOOTER_LEFT_CAN_ID, RIO_CAN_BUS);
    motorRight = new TalonFX(Constants.CanId.SHOOTER_RIGHT_CAN_ID, RIO_CAN_BUS);

    var motorLeftConfig = new TalonFXConfiguration();
    var motorRightConfig = new TalonFXConfiguration();
    motorLeftConfig.Slot0.kP = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KP;
    motorLeftConfig.Slot0.kI = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KI;
    motorLeftConfig.Slot0.kD = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KD;
    motorLeftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorLeftConfig.Slot0.kG = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KG;
    motorLeftConfig.Slot0.kS = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KS;
    motorLeftConfig.Slot0.kV = ShooterTalonFXConstants.PIDConstants.VelocityPIDConstants.SHOOTER_VELOCITY_KV;

    motorLeftConfig.MotorOutput.NeutralMode = ShooterTalonFXConstants.SHOOTER_MECHANISM_NEUTRAL_MODE;
    motorRightConfig.MotorOutput.NeutralMode = ShooterTalonFXConstants.SHOOTER_MECHANISM_NEUTRAL_MODE;

    motorLeftConfig.MotorOutput.Inverted = ShooterTalonFXConstants.SHOOTER_MOTOR_DIRECTION;
    motorRightConfig.MotorOutput.Inverted =
        ShooterTalonFXConstants.SHOOTER_MOTOR_DIRECTION == InvertedValue.Clockwise_Positive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    motorLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motorLeftConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    motorLeftConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_TORQUE_REVERSE_AMP_LIMIT.in(Amps);
    motorRightConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    motorRightConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    motorLeftConfig.CurrentLimits.StatorCurrentLimit =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_STATOR_AMP_LIMIT.in(Amps);
    motorLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true
    motorRightConfig.CurrentLimits.StatorCurrentLimit =
        ShooterTalonFXConstants.MotorSafetyLimits.SHOOTER_STATOR_AMP_LIMIT.in(Amps);
    motorRightConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    PhoenixUtil.tryUntilOk(5, () -> motorLeft.getConfigurator().apply(motorLeftConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> motorRight.getConfigurator().apply(motorRightConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            motorRight.setControl(
                new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed)));

    leftPosition = motorLeft.getPosition();
    rightPosition = motorRight.getPosition();

    leftVelocity = motorLeft.getVelocity();
    rightVelocity = motorRight.getVelocity();

    leftAcceleration = motorLeft.getAcceleration();
    rightAcceleration = motorRight.getAcceleration();

    leftAppliedVoltage = motorLeft.getMotorVoltage();
    rightAppliedVoltage = motorRight.getMotorVoltage();

    leftAppliedCurrent = motorLeft.getStatorCurrent();
    rightAppliedCurrent = motorRight.getStatorCurrent();

    currentPidSlot = motorLeft.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        leftPosition,
        rightPosition,
        leftVelocity,
        rightVelocity,
        leftAcceleration,
        rightAcceleration,
        leftAppliedVoltage,
        rightAppliedVoltage,
        leftAppliedCurrent,
        rightAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(motorLeft, motorRight);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    var leftSignals =
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAcceleration,
            leftAppliedVoltage,
            leftAppliedCurrent,
            currentPidSlot);
    var rightSignals =
        BaseStatusSignal.refreshAll(
            rightPosition,
            rightVelocity,
            rightAcceleration,
            rightAppliedVoltage,
            rightAppliedCurrent);

    inputs.shooterMotorsPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };

    inputs.shooterMotorLeftConnected = leftConnectedDebounce.calculate(leftSignals.isOK());
    inputs.shooterMotorRightConnected = rightConnectedDebounce.calculate(rightSignals.isOK());
    inputs.shooterMotorLeftVelocity = leftVelocity.getValue();
    inputs.shooterMotorRightVelocity = rightVelocity.getValue();
    inputs.shooterMotorLeftAcceleration = leftAcceleration.getValue();
    inputs.shooterMotorRightAcceleration = rightAcceleration.getValue();
    inputs.shooterMotorLeftVolts = leftAppliedVoltage.getValue();
    inputs.shooterMotorRightVolts = rightAppliedVoltage.getValue();
    inputs.shooterMotorLeftCurrent = leftAppliedCurrent.getValue();
    inputs.shooterMotorRightCurrent = rightAppliedCurrent.getValue();
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    if (pidSlot == PIDSlots.OPEN_LOOP)
      throw new IllegalArgumentException(
          "PIDSlots value OPEN_LOOP cannot be used in a closed loop control mode");
    motorLeft.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    setShooterVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setShooterOpenLoop(double percentOutput) {
    motorLeft.setControl(dutyCycleOut.withOutput(percentOutput));
  }

  @Override
  public void stop() {
    motorLeft.stopMotor();
    motorRight.stopMotor();
  }
}
