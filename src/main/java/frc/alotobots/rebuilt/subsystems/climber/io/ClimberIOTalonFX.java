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
package frc.alotobots.rebuilt.subsystems.climber.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants;
import frc.alotobots.rebuilt.subsystems.climber.constants.ClimberTalonFXConstants;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIO.ClimberIOInputs;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIO.PIDSlots;
import frc.alotobots.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climberMotor;
  private final MotionMagicVoltage magicPositionVoltage = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  private StatusSignal<Angle> climberMotorAngle;
  private StatusSignal<AngularVelocity> climberMotorVelocity;
  private StatusSignal<AngularAcceleration> climberMotorAcceleration;
  private StatusSignal<Voltage> climberMotorAppliedVoltage;
  private StatusSignal<Current> climberMotorAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private StatusSignal<ControlModeValue> climberMotorControlMode;
  private Debouncer climberConnectedDebounce = new Debouncer(0.1);

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(Constants.CanId.CLIMBER_CAN_ID, Constants.CanId.RIO_CAN_BUS);

    var climberMotorConfig = new TalonFXConfiguration();

    climberMotorConfig.Slot0.kP = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KP;
    climberMotorConfig.Slot0.kI = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KI;
    climberMotorConfig.Slot0.kD = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KD;
    climberMotorConfig.Slot0.kG = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KG;
    climberMotorConfig.Slot0.kS = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KS;
    climberMotorConfig.Slot0.kV = ClimberTalonFXConstants.PIDConstants.VelocityPIDConstants.KV;
    // PID configuration for position mode (Slot 1)
    climberMotorConfig.Slot1.kP = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KP;
    climberMotorConfig.Slot1.kI = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KI;
    climberMotorConfig.Slot1.kD = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KD;
    climberMotorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    climberMotorConfig.Slot1.kG = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KG;
    climberMotorConfig.Slot1.kS = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KS;
    climberMotorConfig.Slot1.kV = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KV;
    // PID configuration for position MM (Slot 1)
    climberMotorConfig.Slot2.kP = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KP;
    climberMotorConfig.Slot2.kI = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KI;
    climberMotorConfig.Slot2.kD = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KD;
    climberMotorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    climberMotorConfig.Slot2.kG = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KG;
    climberMotorConfig.Slot2.kS = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KS;
    climberMotorConfig.Slot2.kV = ClimberTalonFXConstants.PIDConstants.PositionPIDConstants.KV;

    climberMotorConfig.MotorOutput.NeutralMode = ClimberTalonFXConstants.MECHANISM_NEUTRAL_MODE;

    climberMotorConfig.MotorOutput.Inverted = ClimberTalonFXConstants.MOTOR_DIRECTION;

    climberMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    climberMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ClimberTalonFXConstants.MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    climberMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        ClimberTalonFXConstants.MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    climberMotorConfig.CurrentLimits.StatorCurrentLimit =
        ClimberTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps);
    climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true
    ClimberTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps);
    climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        linearVelocityToTalonFX(ClimberTalonFXConstants.MotionMagicConstants.CRUISE_VELOCITY)
            .in(RotationsPerSecond);
    climberMotorConfig.MotionMagic.MotionMagicAcceleration =
        linearAccelerationToTalonFX(ClimberTalonFXConstants.MotionMagicConstants.ACCELERATION)
            .in(RotationsPerSecondPerSecond);
    climberMotorConfig.MotionMagic.MotionMagicJerk =
        ClimberTalonFXConstants.MotionMagicConstants.JERK;
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberMotorConfig, 0.25));

    climberMotorAngle = climberMotor.getPosition();
    climberMotorVelocity = climberMotor.getVelocity();
    climberMotorAcceleration = climberMotor.getAcceleration();
    climberMotorAppliedVoltage = climberMotor.getMotorVoltage();
    climberMotorAppliedCurrent = climberMotor.getStatorCurrent();
    currentPidSlot = climberMotor.getClosedLoopSlot();
    climberMotorControlMode = climberMotor.getControlMode();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.CanId.DEFAULT_CAN_FREQUENCY,
        climberMotorAngle,
        climberMotorVelocity,
        climberMotorAcceleration,
        climberMotorAppliedVoltage,
        climberMotorAppliedCurrent,
        currentPidSlot,
        climberMotorControlMode);

    ParentDevice.optimizeBusUtilizationForAll(climberMotor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var climberSignals =
        BaseStatusSignal.refreshAll(
            climberMotorAngle,
            climberMotorVelocity,
            climberMotorAcceleration,
            climberMotorAppliedVoltage,
            climberMotorAppliedCurrent,
            currentPidSlot);

    inputs.climberConnected = climberConnectedDebounce.calculate(climberSignals.isOK());

    inputs.climberDistance = talonFXToExtension(climberMotorAngle.getValue());
    inputs.climberAngle = climberMotorAngle.getValue();

    inputs.climberVelocity = climberMotorVelocity.getValue();

    inputs.climberAcceleration = climberMotorAcceleration.getValue();

    inputs.climberVolts = climberMotorAppliedVoltage.getValue();
    inputs.climberCurrent = climberMotorAppliedCurrent.getValue();

    inputs.climberPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.VELOCITY;
          case 1 -> PIDSlots.POSITION;
          case 2 -> PIDSlots.MOTION_MAGIC_POSITION;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };

    talonFXToLinearVelocity(inputs.climberVelocity);
    talonFXToLinearAcceleration(inputs.climberAcceleration);
  }

  @Override
  public void setClimberPosition(Distance position, PIDSlots pidSlot) {
    // if (pidSlot.equals(PIDSlots.MOTION_MAGIC_POSITION)) {
    //   climberMotor.setControl(
    //       magicPositionVoltage
    //           .withPosition(extensionToTalonFX(position))
    //           .withSlot(pidSlot.ordinal()));
    //   return;
    // } else {
    climberMotor.setControl(positionVoltage.withPosition(extensionToTalonFX(position)).withSlot(1));

    // }
  }

  @Override
  public void setClimberPosition(Distance position) {
    setClimberPosition(position, PIDSlots.POSITION);
  }

  @Override
  public void setClimberVelocity(LinearVelocity velocity, PIDSlots pidSlot) {
    climberMotor.setControl(
        velocityVoltage
            .withVelocity(linearVelocityToTalonFX(velocity))
            .withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setClimberVelocity(LinearVelocity velocity) {
    setClimberVelocity(velocity, PIDSlots.VELOCITY);
  }

  @Override
  public void setClimberOpenLoop(double percentOutput) {
    climberMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    climberMotor.stopMotor();
  }

  /**
   * Converts TalonFX rotations to climber extension. TalonFX reports position in rotations in
   * Phoenix 6. Uses regression formula y = EXTENSION_PER_ROTATION + MIN_EXTENSION where x is
   * rotations and y is meters.
   *
   * @param rotations TalonFX motor rotations
   * @return Extension as a Distance unit
   */
  private Distance talonFXToExtension(Angle rotations) {
    return Meters.of(
        ClimberConstants.EXTENSION_PER_ROTATION * rotations.in(Rotations)
            + ClimberConstants.Limits.MIN_CLIMB_EXTENSION.in(Meters));
  }

  /**
   * Converts TalonFX rotational velocity to linear velocity. TalonFX reports velocity in rotations
   * per second in Phoenix 6. Uses slope from regression formula y = EXTENSION_PER_ROTATION +
   * MIN_EXTENSION.
   *
   * @param rotationsPerSecond TalonFX motor rotational velocity
   * @return Linear velocity as a LinearVelocity unit
   */
  private LinearVelocity talonFXToLinearVelocity(AngularVelocity rotationsPerSecond) {
    return MetersPerSecond.of(
        rotationsPerSecond.in(RotationsPerSecond) * ClimberConstants.EXTENSION_PER_ROTATION);
  }

  /**
   * Converts intake extension to TalonFX rotations. TalonFX expects position in rotations in
   * Phoenix 6. Uses inverse of regression formula y = EXTENSION_PER_ROTATION + MIN_EXTENSION,
   * solving for x: x = (y - MIN_EXTENSION) / EXTENSION_PER_ROTATION where y is meters and x is
   * rotations.
   *
   * @param extension Extension as a Distance unit
   * @return TalonFX motor rotations as an Angle unit
   */
  private Angle extensionToTalonFX(Distance extension) {
    return Rotations.of(
        (extension.minus(ClimberConstants.Limits.MIN_CLIMB_EXTENSION).in(Meters))
            / ClimberConstants.EXTENSION_PER_ROTATION);
  }

  /**
   * Converts linear velocity to TalonFX rotational velocity. TalonFX expects velocity in rotations
   * per second in Phoenix 6. Uses inverse slope from regression formula y = EXTENSION_PER_ROTATION
   * + MIN_EXTENSION.
   *
   * @param linearVelocity Linear velocity as a LinearVelocity unit
   * @return TalonFX motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity linearVelocityToTalonFX(LinearVelocity linearVelocity) {
    return RotationsPerSecond.of(
        linearVelocity.in(MetersPerSecond) / ClimberConstants.EXTENSION_PER_ROTATION);
  }

  /**
   * Converts linear acceleration to TalonFX rotational acceleration. TalonFX expects acceleration
   * in rotations per second squared in Phoenix 6. Uses the same conversion factor as velocity since
   * acceleration is the time derivative of velocity.
   *
   * @param linearAcceleration Linear acceleration as a LinearAcceleration unit
   * @return TalonFX motor rotational acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration linearAccelerationToTalonFX(LinearAcceleration linearAcceleration) {
    return RotationsPerSecondPerSecond.of(
        linearAcceleration.in(MetersPerSecondPerSecond) / ClimberConstants.EXTENSION_PER_ROTATION);
  }

  /**
   * Converts TalonFX rotational acceleration to linear acceleration. TalonFX reports acceleration
   * in rotations per second squared in Phoenix 6. Uses the same conversion factor as velocity since
   * acceleration is the time derivative of velocity.
   *
   * @param rotationalAcceleration TalonFX motor rotational acceleration as an AngularAcceleration
   *     unit
   * @return Linear acceleration as a LinearAcceleration unit
   */
  private LinearAcceleration talonFXToLinearAcceleration(
      AngularAcceleration rotationalAcceleration) {
    return MetersPerSecondPerSecond.of(
        rotationalAcceleration.in(RotationsPerSecondPerSecond)
            * ClimberConstants.EXTENSION_PER_ROTATION);
  }
}
