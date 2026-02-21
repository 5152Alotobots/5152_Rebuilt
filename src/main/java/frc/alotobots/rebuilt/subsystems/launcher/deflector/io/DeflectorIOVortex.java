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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorVortexConstants;
import org.littletonrobotics.junction.Logger;

public class DeflectorIOVortex implements DeflectorIO {
  private final SparkFlex deflectorMotor;
  private final CANcoder deflectorEncoder;

  private final DigitalInput backLimitSwitch;
  private final Debouncer backLimitDebouncer;
  private final Debouncer deflectorMotorConnectedDebouncer;

  // CANcoder Signals
  private final StatusSignal<Angle> encoderPosition;
  private final StatusSignal<AngularVelocity> encoderVelocity;

  public DeflectorIOVortex() {
    deflectorMotor = new SparkFlex(Constants.CanId.DEFLECTOR_MOTOR_CAN_ID, MotorType.kBrushless);
    deflectorEncoder = new CANcoder(Constants.CanId.DEFLECTOR_ENCODER_CAN_ID, "rio");

    deflectorMotorConnectedDebouncer = new Debouncer(0.5);
    backLimitDebouncer = new Debouncer(0.1);
    backLimitSwitch = new DigitalInput(0);

    SparkFlexConfig deflectorMotorConfig = new SparkFlexConfig();

    deflectorMotorConfig.idleMode(IdleMode.kCoast).inverted(true);

    deflectorMotorConfig
        .encoder
        .positionConversionFactor(DeflectorVortexConstants.SENSOR_TO_MECHANISM_RATIO)
        .velocityConversionFactor(DeflectorVortexConstants.SENSOR_TO_MECHANISM_RATIO);

    deflectorMotorConfig.closedLoop.pid(
        DeflectorVortexConstants.POSITION_P_GAIN,
        DeflectorVortexConstants.POSITION_I_GAIN,
        DeflectorVortexConstants.POSITION_D_GAIN,
        ClosedLoopSlot.kSlot0);

    // Write config to the motor controller
    deflectorMotor.configure(
        deflectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoderPosition = deflectorEncoder.getAbsolutePosition();
    encoderVelocity = deflectorEncoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, encoderPosition, encoderVelocity);
    deflectorEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(
            deflectorMotor.hasActiveFault() == false // Basic connection/fault check
            );

    inputs.deflectorMotorPosition = Rotations.of(deflectorMotor.getEncoder().getPosition());
    inputs.deflectorMotorVelocity =
        RotationsPerSecond.of(deflectorMotor.getEncoder().getVelocity());
    inputs.deflectorPosition =
        Rotations.of(
            inputs.deflectorMotorPosition.in(Rotations)
                / DeflectorVortexConstants.SENSOR_TO_MECHANISM_RATIO);

    inputs.deflectorMotorVolts =
        Volts.of(deflectorMotor.getAppliedOutput() * deflectorMotor.getBusVoltage());
    inputs.deflectorMotorCurrent = Amps.of(deflectorMotor.getOutputCurrent());

    inputs.deflectorEncoderPosition = encoderPosition.getValue();
    inputs.deflectorEncoderVelocity = encoderVelocity.getValue();
  }

  public void seedInternalEncoder() {
    // Wait a moment for the CANcoder to send its initial data over the bus
    Angle absolutePosition = deflectorEncoder.getAbsolutePosition().getValue();

    // Tell the Vortex internal encoder that its current position is the CANcoder's absolute
    // position
    deflectorMotor.getEncoder().setPosition(absolutePosition.in(Rotations));
  }

  @Override
  public void setDeflectorPosition(Angle position) {
    setDeflectorPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        pidSlot == PIDSlots.VELOCITY ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;

    deflectorMotor
        .getClosedLoopController()
        .setReference(position.in(Rotations), ControlType.kPosition, slot);
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        pidSlot == PIDSlots.VELOCITY ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;

    deflectorMotor
        .getClosedLoopController()
        .setReference(
            velocity.in(RotationsPerSecond), // Adjust time unit as needed
            ControlType.kVelocity,
            slot);
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity) {
    setDeflectorVelocity(velocity, PIDSlots.VELOCITY);
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
