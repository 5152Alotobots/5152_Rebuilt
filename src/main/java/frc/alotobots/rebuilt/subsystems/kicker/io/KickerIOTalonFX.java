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
package frc.alotobots.rebuilt.subsystems.kicker.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX motorKicker;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private StatusSignal<Angle> kickerPosition;
  private StatusSignal<AngularVelocity> kickerVelocity;
  private StatusSignal<AngularAcceleration> kickerAcceleration;
  private StatusSignal<Voltage> kickerAppliedVoltage;
  private StatusSignal<Current> kickerAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private Debouncer kickerConnectedDebounce = new Debouncer(0.1);

  public KickerIOTalonFX() {
    motorKicker = new TalonFX(Constants.CanId.KICKER_CAN_ID, RIO_CAN_BUS);

    var motorKickerConfig = new TalonFXConfiguration();
    motorKickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorKickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorKickerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorKickerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        KickerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorKickerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        KickerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorKickerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        KickerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorKickerConfig.Slot0.kP = KickerTalonFXConstants.VELOCITY_P_GAIN;
    motorKickerConfig.Slot0.kI = KickerTalonFXConstants.VELOCITY_I_GAIN;
    motorKickerConfig.Slot0.kD = KickerTalonFXConstants.VELOCITY_D_GAIN;
    motorKickerConfig.Slot0.kV = KickerTalonFXConstants.VELOCITY_V_GAIN;
    motorKickerConfig.Slot0.kS = KickerTalonFXConstants.VELOCITY_S_GAIN;

    motorKickerConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    motorKickerConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_TORQUE_REVERSE_AMP_LIMIT.in(Amps);
    motorKickerConfig.CurrentLimits.StatorCurrentLimit =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_STATOR_AMP_LIMIT.in(Amps);
    motorKickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorKickerConfig.CurrentLimits.SupplyCurrentLimit =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_SUPPLY_PEAK_LIMIT.in(Amps);
    motorKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorKickerConfig.CurrentLimits.SupplyCurrentLowerLimit =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_SUPPLY_SUSTAINED_LIMIT.in(Amps);
    motorKickerConfig.CurrentLimits.SupplyCurrentLowerTime =
        KickerTalonFXConstants.MotorSafetyLimits.KICKER_SUPPLY_PEAK_DURATION.in(Seconds);

    PhoenixUtil.tryUntilOk(5, () -> motorKicker.getConfigurator().apply(motorKickerConfig, 0.25));

    kickerPosition = motorKicker.getPosition();
    kickerVelocity = motorKicker.getVelocity();
    kickerAcceleration = motorKicker.getAcceleration();
    kickerAppliedVoltage = motorKicker.getMotorVoltage();
    kickerAppliedCurrent = motorKicker.getStatorCurrent();
    currentPidSlot = motorKicker.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        kickerPosition,
        kickerVelocity,
        kickerAcceleration,
        kickerAppliedVoltage,
        kickerAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(motorKicker);
  }

  @Override
  public void updateInputs(KickerIO.KickerIOInputs inputs) {
    var kickerSignals =
        BaseStatusSignal.refreshAll(
            kickerPosition,
            kickerVelocity,
            kickerAcceleration,
            kickerAppliedVoltage,
            kickerAppliedCurrent);

    inputs.kickerMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot.getValue());
        };

    inputs.kickerMotorConnected = kickerConnectedDebounce.calculate(kickerSignals.isOK());
    inputs.kickerMotorVelocity = kickerVelocity.getValue();
    inputs.kickerMotorAcceleration = kickerAcceleration.getValue();
    inputs.kickerMotorVolts = kickerAppliedVoltage.getValue();
    inputs.kickerMotorCurrent = kickerAppliedCurrent.getValue();

    inputs.kickerMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot.getValue());
        };
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    motorKicker.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    setKickerVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setKickerOpenLoop(double percentOutput) {
    motorKicker.setControl(dutyCycleOut.withOutput(1));
  }

  @Override
  public void stop() {
    motorKicker.stopMotor();
  }
}
