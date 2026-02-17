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

import static edu.wpi.first.units.Units.Amps;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final CANBus canBus = new CANBus("rio");
  private final TalonFX intakeRollerMotor;

  private StatusSignal<AngularVelocity> intakeRollerVelocity;
  private StatusSignal<Voltage> intakeRollerAppliedVoltage;
  private StatusSignal<Current> intakeRollerAppliedCurrent;
  private Debouncer intakeRollerConnectedDebounce = new Debouncer(0.1);

  public IntakeRollerIOTalonFX() {
    intakeRollerMotor = new TalonFX(Constants.CanId.INTAKE_ROLLER_CAN_ID, canBus);

    var intakeRollerMotorConfig = new TalonFXConfiguration();
    intakeRollerMotorConfig.MotorOutput.NeutralMode = IntakeRollerTalonFXConstants.MECHANISM_NEUTRAL_MODE;
    intakeRollerMotorConfig.MotorOutput.Inverted = IntakeRollerTalonFXConstants.MOTOR_DIRECTION;
    intakeRollerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    intakeRollerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            IntakeRollerTalonFXConstants.MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    intakeRollerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            IntakeRollerTalonFXConstants.MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    intakeRollerMotorConfig.CurrentLimits.StatorCurrentLimit =
            IntakeRollerTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps);
    intakeRollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    PhoenixUtil.tryUntilOk(
        5, () -> intakeRollerMotor.getConfigurator().apply(intakeRollerMotorConfig, 0.25));

    intakeRollerVelocity = intakeRollerMotor.getVelocity();
    intakeRollerAppliedVoltage = intakeRollerMotor.getMotorVoltage();
    intakeRollerAppliedCurrent = intakeRollerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeRollerVelocity,
        intakeRollerAppliedVoltage,
        intakeRollerAppliedCurrent);
    
    ParentDevice.optimizeBusUtilizationForAll(intakeRollerMotor);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    var intakeRollerSignals =
        BaseStatusSignal.refreshAll(
            intakeRollerVelocity,
            intakeRollerAppliedVoltage,
            intakeRollerAppliedCurrent);

    inputs.intakeRollerMotorConnected =
        intakeRollerConnectedDebounce.calculate(intakeRollerSignals.isOK());
    inputs.intakeRollerMotorVelocity = intakeRollerVelocity.getValue();
    inputs.intakeRollerMotorVolts = intakeRollerAppliedVoltage.getValue();
    inputs.intakeRollerMotorCurrent = intakeRollerAppliedCurrent.getValue();
    
  }

  @Override
  public void setIntakeRollerOpenLoop(double percentOutput) {
    intakeRollerMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    intakeRollerMotor.stopMotor();
  }
}
