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
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.MAX_ANGLE;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorVortexConstants.DEFLECTOR_ROTATION_PER_ROTATION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
  private final Debouncer deflectorEncoderConnectedDebouncer;

  // CANcoder Signals
  private final StatusSignal<Angle> encoderPosition;
  private final StatusSignal<AngularVelocity> encoderVelocity;

  public DeflectorIOVortex() {
    deflectorMotor = new SparkFlex(Constants.CanId.DEFLECTOR_MOTOR_CAN_ID, MotorType.kBrushless);
    deflectorEncoder = new CANcoder(Constants.CanId.DEFLECTOR_ENCODER_CAN_ID, RIO_CAN_BUS);

    deflectorMotorConnectedDebouncer = new Debouncer(0.1);
    deflectorEncoderConnectedDebouncer = new Debouncer(0.1);
    backLimitDebouncer = new Debouncer(0.5);
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

    deflectorMotorConfig.idleMode(DeflectorVortexConstants.MECHANISM_NEUTRAL_MODE);
    deflectorMotorConfig.inverted(DeflectorVortexConstants.MOTOR_DIRECTION_INVERTED);
    deflectorMotorConfig.smartCurrentLimit(
        (int) DeflectorVortexConstants.MotorSafetyLimits.TORQUE_AMP_LIMIT.in(Amps));

    // Position (Slot 0)
    deflectorMotorConfig.closedLoop.p(
        DeflectorVortexConstants.PIDConstants.PositionPIDConstants.KP, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.i(
        DeflectorVortexConstants.PIDConstants.PositionPIDConstants.KI, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.d(
        DeflectorVortexConstants.PIDConstants.PositionPIDConstants.KD, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.feedForward.kG(
        DeflectorVortexConstants.PIDConstants.PositionPIDConstants.KG, ClosedLoopSlot.kSlot0);
    deflectorMotorConfig.closedLoop.allowedClosedLoopError(
        DeflectorVortexConstants.PIDConstants.PositionPIDConstants.ALLOWED_CLOSED_LOOP_ERROR,
        ClosedLoopSlot.kSlot0);

    // Velocity (Slot 1)
    deflectorMotorConfig.closedLoop.p(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KP, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.i(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KI, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.d(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KD, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.feedForward.kG(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KG, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.feedForward.kS(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KS, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.feedForward.kV(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.KV, ClosedLoopSlot.kSlot1);
    deflectorMotorConfig.closedLoop.allowedClosedLoopError(
        DeflectorVortexConstants.PIDConstants.VelocityPIDConstants.ALLOWED_CLOSED_LOOP_ERROR,
        ClosedLoopSlot.kSlot1);

    deflectorMotor.configure(
        deflectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoderPosition = deflectorEncoder.getPosition();
    encoderVelocity = deflectorEncoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY, encoderPosition, encoderVelocity);
    ParentDevice.optimizeBusUtilizationForAll(deflectorEncoder);

    // Seed the internal encoder of the vortex using the CANCoder's absolute position
    seedVortexInternalEncoder();
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    var deflectorEncoderSignals = BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(deflectorMotor.getBusVoltage() > 0);
    inputs.deflectorEncoderConnected =
        deflectorEncoderConnectedDebouncer.calculate(deflectorEncoderSignals.isOK());

    inputs.deflectorMotorAngle = Rotations.of(deflectorMotor.getEncoder().getPosition());
    inputs.deflectorEncoderAngle = encoderPosition.getValue();
    inputs.deflectorAngle =
        vortexToDeflectorAngle(Rotations.of(deflectorMotor.getEncoder().getPosition()));

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

  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        switch (pidSlot) {
          case DEFAULT_POSITION -> ClosedLoopSlot.kSlot0;
          case VELOCITY -> ClosedLoopSlot.kSlot1;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + pidSlot.ordinal());
        };

    deflectorMotor
        .getClosedLoopController()
        .setSetpoint(deflectorAngleToVortex(position).in(Rotations), ControlType.kPosition, slot);
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
            deflectorAngularVelocityToVortex(velocity).in(RotationsPerSecond),
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

  public void seedVortexInternalEncoder() {
    Angle absolutePosition = deflectorEncoder.getPosition().getValue();

    // Tell the Vortex internal encoder that its current position is the CANcoder's absolute
    // position
    deflectorMotor.getEncoder().setPosition(absolutePosition.in(Rotations));
  }

  /**
   * Converts Vortex motor position to deflector angle. Uses regression formula y = SLOPE * x +
   * MIN_ANGLE where x is motor position in radians and y is deflector angle in radians.
   *
   * @param motorPosition Vortex motor position as an Angle unit
   * @return Deflector angle as an Angle unit
   */
  private Angle vortexToDeflectorAngle(Angle motorPosition) {
    return Radians.of(
        DEFLECTOR_ROTATION_PER_ROTATION * motorPosition.in(Radians) + MAX_ANGLE.in(Radians));
  }

  /**
   * Converts Vortex motor velocity to deflector angular velocity. Uses the slope from the
   * regression formula y = SLOPE * x + MIN_ANGLE as the conversion factor.
   *
   * @param motorVelocity Vortex motor rotational velocity as an AngularVelocity unit
   * @return Deflector angular velocity as an AngularVelocity unit
   */
  private AngularVelocity vortexToDeflectorAngularVelocity(AngularVelocity motorVelocity) {
    return RadiansPerSecond.of(
        motorVelocity.in(RadiansPerSecond) * DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angle to Vortex motor position. Uses inverse of regression formula y = SLOPE
   * * x + MIN_ANGLE, solving for x: x = (y - MIN_ANGLE) / SLOPE where y is deflector angle in
   * radians and x is motor position in radians.
   *
   * @param deflectorAngle Deflector angle as an Angle unit
   * @return Vortex motor position as an Angle unit
   */
  private Angle deflectorAngleToVortex(Angle deflectorAngle) {
    return Radians.of(
        (deflectorAngle.in(Radians) - MAX_ANGLE.in(Radians)) / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angular velocity to Vortex motor velocity. Uses inverse slope from
   * regression formula y = SLOPE * x + MIN_ANGLE.
   *
   * @param deflectorAngularVelocity Deflector angular velocity as an AngularVelocity unit
   * @return Vortex motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity deflectorAngularVelocityToVortex(
      AngularVelocity deflectorAngularVelocity) {
    return RadiansPerSecond.of(
        deflectorAngularVelocity.in(RadiansPerSecond) / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angular acceleration to Vortex motor rotational acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param deflectorAngularAcceleration Deflector angular acceleration as an AngularAcceleration
   *     unit
   * @return Vortex motor rotational acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration deflectorAngularAccelerationToVortex(
      AngularAcceleration deflectorAngularAcceleration) {
    return RadiansPerSecondPerSecond.of(
        deflectorAngularAcceleration.in(RadiansPerSecondPerSecond)
            / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts Vortex motor rotational acceleration to deflector angular acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param motorAcceleration Vortex motor rotational acceleration as an AngularAcceleration unit
   * @return Deflector angular acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration vortexToDeflectorAngularAcceleration(
      AngularAcceleration motorAcceleration) {
    return RadiansPerSecondPerSecond.of(
        motorAcceleration.in(RadiansPerSecondPerSecond) * DEFLECTOR_ROTATION_PER_ROTATION);
  }
}
