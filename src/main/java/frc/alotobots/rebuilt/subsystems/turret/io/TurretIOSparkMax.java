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

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants.CanId;
import frc.alotobots.rebuilt.subsystems.turret.constants.TurretSparkMaxConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOSparkMax implements TurretIO {
  private final SparkMax turretMotor;

  private final DigitalInput cwLimitSwitch;
  private final DigitalInput ccwLimitSwitch;

  private final Debouncer cwLimitDebouncer;
  private final Debouncer ccwLimitDebouncer;
  private final Debouncer turretMotorConectedDebouncer;

  private final SparkClosedLoopController closedLoopController;

  public TurretIOSparkMax() {
    turretMotor = new SparkMax(CanId.TURRET_CAN_ID, SparkMax.MotorType.kBrushless);
    cwLimitSwitch = new DigitalInput(0);
    ccwLimitSwitch = new DigitalInput(1);
    cwLimitDebouncer = new Debouncer(0.1);
    ccwLimitDebouncer = new Debouncer(0.1);
    turretMotorConectedDebouncer = new Debouncer(0.5);
    closedLoopController = turretMotor.getClosedLoopController();

    // Initialize SparkMax turret motor
    var turretMotorConfig = new SparkMaxConfig();
    var turretPositionPIDSlot =
        ClosedLoopSlot.fromInt(TurretIO.PIDSlots.DEFAULT_POSITION.ordinal());

    turretMotorConfig.closedLoop.pid(.5, 0, 0, turretPositionPIDSlot);
    turretMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    turretMotorConfig.signals.primaryEncoderVelocityAlwaysOn(true);

    turretMotorConfig.signals.primaryEncoderPositionPeriodMs(20);
    turretMotorConfig.signals.primaryEncoderVelocityPeriodMs(20);

    turretMotorConfig.signals.motorTemperaturePeriodMs(1000);

    turretMotorConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    turretMotorConfig.signals.absoluteEncoderVelocityAlwaysOn(true);

    turretMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
    turretMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);

    turretMotorConfig.idleMode(TurretSparkMaxConstants.NEUTRAL_MODE);
    turretMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    turretMotorConfig.signals.primaryEncoderVelocityAlwaysOn(true);
    turretMotorConfig.absoluteEncoder.zeroOffset(
        TurretSparkMaxConstants.ABSOLUTE_ENCODER_ZERO_OFFSET);
    turretMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    turretMotor.configure(
        turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Read data from SparkMax and update inputs
    inputs.turretMotorConnected =
        turretMotorConectedDebouncer.calculate(turretMotor.getLastError().equals(REVLibError.kOk));
    inputs.cwLimit = cwLimitDebouncer.calculate(cwLimitSwitch.get());
    inputs.ccwLimit = ccwLimitDebouncer.calculate(ccwLimitSwitch.get());

    inputs.turretMotorPosition =
        Angle.ofBaseUnits(turretMotor.getAbsoluteEncoder().getPosition(), Rotations);
    inputs.turretMotorVelocity =
        AngularVelocity.ofBaseUnits(
            turretMotor.getAbsoluteEncoder().getVelocity(), RotationsPerSecond);
    inputs.turretMotorVolts =
        Volt.ofBaseUnits(turretMotor.getAppliedOutput() * turretMotor.getBusVoltage());
    inputs.turretMotorCurrent = Amps.ofBaseUnits(turretMotor.getOutputCurrent());
  }

  @Override
  public void setTurretPosition(Angle position) {
    setTurretPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  @Override
  public void setTurretPosition(Angle position, PIDSlots pidSlot) {
    // Set SparkMax to position control mode and move to target position
    Logger.recordOutput("turretIO/setpoint", position);
    closedLoopController.setSetpoint(
        position.in(Rotations), ControlType.kPosition, ClosedLoopSlot.fromInt(pidSlot.ordinal()));
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
