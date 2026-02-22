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

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.MAX_ANGLE;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorVortexConstants.SENSOR_TO_MECHANISM_RATIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorVortexConstants;
import frc.alotobots.util.PhoenixUtil;
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
    deflectorEncoder =
        new CANcoder(
            Constants.CanId.DEFLECTOR_ENCODER_CAN_ID, DeflectorVortexConstants.DEFLECTOR_CAN_BUS);

    deflectorMotorConnectedDebouncer = new Debouncer(0.5);
    backLimitDebouncer = new Debouncer(0.1);
    backLimitSwitch = null;

    SparkFlexConfig deflectorMotorConfig = new SparkFlexConfig();
    CANcoderConfiguration deflectorEncoderConfig = new CANcoderConfiguration();

    deflectorEncoderConfig.MagnetSensor.MagnetOffset =
        DeflectorVortexConstants.ENCODER_MAGNET_OFFSET;
    deflectorEncoderConfig.MagnetSensor.SensorDirection =
        DeflectorVortexConstants.ENCODER_SENSOR_DIRECTION;
    deflectorEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        DeflectorVortexConstants.ABSOLUTE_SENSOR_DISCONTINUITY_POINT;

    PhoenixUtil.tryUntilOk(
        5, () -> deflectorEncoder.getConfigurator().apply(deflectorEncoderConfig, 0.25));

    deflectorMotorConfig.idleMode(IdleMode.kBrake).inverted(false);

    deflectorMotorConfig.closedLoop.p(
        DeflectorVortexConstants.POSITION_P_GAIN, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.i(
        DeflectorVortexConstants.POSITION_I_GAIN, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.d(
        DeflectorVortexConstants.POSITION_D_GAIN, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.allowedClosedLoopError(.1, ClosedLoopSlot.kSlot0);

    deflectorMotor.configure(
        deflectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoderPosition = deflectorEncoder.getPosition();
    encoderVelocity = deflectorEncoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, encoderPosition, encoderVelocity);
    deflectorEncoder.optimizeBusUtilization();
    seedInternalEncoder();
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {

    BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(deflectorMotor.getBusVoltage() > 0);
    inputs.deflectorMotorPosition = Rotations.of(deflectorMotor.getEncoder().getPosition());
    inputs.deflectorEncoderPosition = encoderPosition.getValue();
    inputs.deflectorPosition =
        vortexToHoodAngle(Rotations.of(deflectorMotor.getEncoder().getPosition()));
    inputs.deflectorEncoderVelocity = encoderVelocity.getValue();
    inputs.deflectorMotorVelocity =
        RotationsPerSecond.of(deflectorMotor.getEncoder().getVelocity());
    // No deflector motor acceleration because REV is shit
    inputs.deflectorMotorVolts =
        Volts.of(deflectorMotor.getAppliedOutput() * deflectorMotor.getBusVoltage());
    inputs.deflectorMotorCurrent = Amps.of(deflectorMotor.getOutputCurrent());
    // TODO: Limit switch goes here
  }

  @Override
  public void setDeflectorPosition(Angle position) {
    setDeflectorPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  // IMPORTANT Pos takes values from -6.28 - 0
  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        pidSlot == PIDSlots.VELOCITY ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;

    deflectorMotor
        .getClosedLoopController()
        .setSetpoint(hoodAngleToVortex(position).in(Rotations), ControlType.kPosition, slot);
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        switch (pidSlot) {
          case DEFAULT_POSITION -> ClosedLoopSlot.kSlot0;
          case VELOCITY -> ClosedLoopSlot.kSlot1;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + pidSlot.ordinal());
        };

    deflectorMotor
        .getClosedLoopController()
        .setSetpoint(
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
    Logger.recordOutput("Launcher/Deflector/openLoopPercentOut", percentOutput);
    deflectorMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    deflectorMotor.stopMotor();
  }

  public void seedInternalEncoder() {
    Angle absolutePosition = deflectorEncoder.getPosition().getValue();

    // Tell the Vortex internal encoder that its current position is the CANcoder's absolute
    // position
    deflectorMotor.getEncoder().setPosition(absolutePosition.in(Rotations));
  }

  /**
   * Converts Vortex motor position to hood angle. Uses regression formula y = SLOPE * x + MIN_ANGLE
   * where x is motor position in radians and y is hood angle in radians.
   *
   * @param motorPosition Vortex motor position as an Angle unit
   * @return Hood angle as an Angle unit
   */
  private Angle vortexToHoodAngle(Angle motorPosition) {
    return Radians.of(
        SENSOR_TO_MECHANISM_RATIO * motorPosition.in(Radians) + MAX_ANGLE.in(Radians));
  }

  /**
   * Converts Vortex motor velocity to hood angular velocity. Uses the slope from the regression
   * formula y = SLOPE * x + MIN_ANGLE as the conversion factor.
   *
   * @param motorVelocity Vortex motor rotational velocity as an AngularVelocity unit
   * @return Hood angular velocity as an AngularVelocity unit
   */
  private AngularVelocity vortexToHoodAngularVelocity(AngularVelocity motorVelocity) {
    return RadiansPerSecond.of(motorVelocity.in(RadiansPerSecond) * SENSOR_TO_MECHANISM_RATIO);
  }

  /**
   * Converts hood angle to Vortex motor position. Uses inverse of regression formula y = SLOPE * x
   * + MIN_ANGLE, solving for x: x = (y - MIN_ANGLE) / SLOPE where y is hood angle in radians and x
   * is motor position in radians.
   *
   * @param hoodAngle Hood angle as an Angle unit
   * @return Vortex motor position as an Angle unit
   */
  private Angle hoodAngleToVortex(Angle hoodAngle) {
    return Radians.of((hoodAngle.in(Radians) - MAX_ANGLE.in(Radians)) / SENSOR_TO_MECHANISM_RATIO);
  }

  /**
   * Converts hood angular velocity to Vortex motor velocity. Uses inverse slope from regression
   * formula y = SLOPE * x + MIN_ANGLE.
   *
   * @param hoodAngularVelocity Hood angular velocity as an AngularVelocity unit
   * @return Vortex motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity hoodAngularVelocityToVortex(AngularVelocity hoodAngularVelocity) {
    return RadiansPerSecond.of(
        hoodAngularVelocity.in(RadiansPerSecond) / SENSOR_TO_MECHANISM_RATIO);
  }

  /**
   * Converts hood angular acceleration to Vortex motor rotational acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param hoodAngularAcceleration Hood angular acceleration as an AngularAcceleration unit
   * @return Vortex motor rotational acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration hoodAngularAccelerationToVortex(
      AngularAcceleration hoodAngularAcceleration) {
    return RadiansPerSecondPerSecond.of(
        hoodAngularAcceleration.in(RadiansPerSecondPerSecond) / SENSOR_TO_MECHANISM_RATIO);
  }

  /**
   * Converts Vortex motor rotational acceleration to hood angular acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param motorAcceleration Vortex motor rotational acceleration as an AngularAcceleration unit
   * @return Hood angular acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration vortexToHoodAngularAcceleration(
      AngularAcceleration motorAcceleration) {
    return RadiansPerSecondPerSecond.of(
        motorAcceleration.in(RadiansPerSecondPerSecond) * SENSOR_TO_MECHANISM_RATIO);
  }
}
