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
package frc.alotobots.rebuilt.subsystems.intake.roller.io;

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
import edu.wpi.first.units.measure.*;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;
import org.dyn4j.exception.ArgumentNullException;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final CANBus canBus = new CANBus("rio");
  private final TalonFX intakeRollerMotor;
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private StatusSignal<Angle> intakeRollerPosition;
  private StatusSignal<AngularVelocity> intakeRollerVelocity;
  private StatusSignal<AngularAcceleration> intakeRollerAcceleration;
  private StatusSignal<Voltage> intakeRollerAppliedVoltage;
  private StatusSignal<Current> intakeRollerAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private Debouncer intakeRollerConnectedDebounce = new Debouncer(0.1);

  public IntakeRollerIOTalonFX() {
    intakeRollerMotor = new TalonFX(Constants.CanId.INTAKE_ROLLER_CAN_ID, canBus);

    var intakeRollerMotorConfig = new TalonFXConfiguration();
    intakeRollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeRollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeRollerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakeRollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        IntakeRollerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    intakeRollerMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        IntakeRollerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    intakeRollerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        IntakeRollerTalonFXConstants.CLOSED_LOOP_RAMP_RATE;
    intakeRollerMotorConfig.Slot0.kP = IntakeRollerTalonFXConstants.VELOCITY_P_GAIN;
    intakeRollerMotorConfig.Slot0.kI = IntakeRollerTalonFXConstants.VELOCITY_I_GAIN;
    intakeRollerMotorConfig.Slot0.kD = IntakeRollerTalonFXConstants.VELOCITY_D_GAIN;
    intakeRollerMotorConfig.Slot0.kV = IntakeRollerTalonFXConstants.VELOCITY_V_GAIN;
    intakeRollerMotorConfig.Slot0.kS = IntakeRollerTalonFXConstants.VELOCITY_S_GAIN;

    PhoenixUtil.tryUntilOk(
        5, () -> intakeRollerMotor.getConfigurator().apply(intakeRollerMotorConfig, 0.25));

    intakeRollerPosition = intakeRollerMotor.getPosition();
    intakeRollerVelocity = intakeRollerMotor.getVelocity();
    intakeRollerAcceleration = intakeRollerMotor.getAcceleration();
    intakeRollerAppliedVoltage = intakeRollerMotor.getMotorVoltage();
    intakeRollerAppliedCurrent = intakeRollerMotor.getStatorCurrent();
    currentPidSlot = intakeRollerMotor.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeRollerPosition,
        intakeRollerVelocity,
        intakeRollerAcceleration,
        intakeRollerAppliedVoltage,
        intakeRollerAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(intakeRollerMotor);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    var intakeRollerSignals =
        BaseStatusSignal.refreshAll(
            intakeRollerPosition,
            intakeRollerVelocity,
            intakeRollerAcceleration,
            intakeRollerAppliedVoltage,
            intakeRollerAppliedCurrent);

    inputs.intakeRollerMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new ArgumentNullException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };

    inputs.intakeRollerMotorConnected =
        intakeRollerConnectedDebounce.calculate(intakeRollerSignals.isOK());
    inputs.intakeRollerMotorVelocity = intakeRollerVelocity.getValue();
    inputs.intakeRollerMotorAcceleration = intakeRollerAcceleration.getValue();
    inputs.intakeRollerMotorVolts = intakeRollerAppliedVoltage.getValue();
    inputs.intakeRollerMotorCurrent = intakeRollerAppliedCurrent.getValue();

    inputs.intakeRollerMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_VELOCITY;
          default -> throw new ArgumentNullException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };
  }

  @Override
  public void setIntakeRollerVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    intakeRollerMotor.setControl(
        velocityVoltage.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setIntakeRollerVelocity(AngularVelocity velocity) {
    setIntakeRollerVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setIntakeRollerOpenLoop(double percentOutput) {
    intakeRollerMotor.setControl(dutyCycleOut.withOutput(1));
  }

  @Override
  public void stop() {
    intakeRollerMotor.stopMotor();
  }
}
