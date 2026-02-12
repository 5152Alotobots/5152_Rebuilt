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
package frc.alotobots.rebuilt.subsystems.belt.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;

public class BeltIOTalonFX implements BeltIO {
  private final CANBus canBus = new CANBus("rio");
  private final TalonFX motorBelt;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private StatusSignal<Angle> beltPosition;
  private StatusSignal<AngularVelocity> beltVelocity;
  private StatusSignal<AngularAcceleration> beltAcceleration;
  private StatusSignal<Voltage> beltAppliedVoltage;
  private StatusSignal<Current> beltAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private Debouncer beltConnectedDebounce = new Debouncer(0.1);

  public BeltIOTalonFX() {
    motorBelt = new TalonFX(Constants.CanId.BELT_KICKER_CAN_ID, canBus);

    var motorBeltConfig = new TalonFXConfiguration();
    motorBeltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorBeltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorBeltConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorBeltConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        BeltTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorBeltConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        BeltTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorBeltConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        BeltTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    motorBeltConfig.Slot0.kP = BeltTalonFXConstants.VELOCITY_P_GAIN;
    motorBeltConfig.Slot0.kI = BeltTalonFXConstants.VELOCITY_I_GAIN;
    motorBeltConfig.Slot0.kD = BeltTalonFXConstants.VELOCITY_D_GAIN;
    motorBeltConfig.Slot0.kV = BeltTalonFXConstants.VELOCITY_V_GAIN;
    motorBeltConfig.Slot0.kS = BeltTalonFXConstants.VELOCITY_S_GAIN;

    PhoenixUtil.tryUntilOk(5, () -> motorBelt.getConfigurator().apply(motorBeltConfig, 0.25));

    beltPosition = motorBelt.getPosition();
    beltVelocity = motorBelt.getVelocity();
    beltAcceleration = motorBelt.getAcceleration();
    beltAppliedVoltage = motorBelt.getMotorVoltage();
    beltAppliedCurrent = motorBelt.getStatorCurrent();
    currentPidSlot = motorBelt.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        beltPosition,
        beltVelocity,
        beltAcceleration,
        beltAppliedVoltage,
        beltAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(motorBelt);
  }

  @Override
  public void updateInputs(BeltIOInputs inputs) {
    var beltSignals =
        BaseStatusSignal.refreshAll(
            beltPosition, beltVelocity, beltAcceleration, beltAppliedVoltage, beltAppliedCurrent);

    inputs.beltMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot.getValue());
        };

    inputs.beltMotorConnected = beltConnectedDebounce.calculate(beltSignals.isOK());
    inputs.beltMotorVelocity = beltVelocity.getValue();
    inputs.beltMotorAcceleration = beltAcceleration.getValue();
    inputs.beltMotorVolts = beltAppliedVoltage.getValue();
    inputs.beltMotorCurrent = beltAppliedCurrent.getValue();

    inputs.beltMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new IllegalStateException(
              "Bad things happened in Shooter and You check if you set the PID SLOTS RIGHT"
                  + currentPidSlot.getValue());
        };
  }

  @Override
  public void setBeltVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    motorBelt.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setBeltVelocity(AngularVelocity velocity) {
    setBeltVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setBeltOpenLoop(double percentOutput) {
    motorBelt.setControl(dutyCycleOut.withOutput(percentOutput));
  }

  @Override
  public void stop() {
    motorBelt.stopMotor();
  }
}
