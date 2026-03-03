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
package frc.alotobots.rebuilt.subsystems.intake.extendo.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Limits.MIN_EXTENSION;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoTalonFXConstants.EXTENSION_PER_ROTATION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoTalonFXConstants;
import frc.alotobots.util.PhoenixUtil;

public class IntakeExtendoIOTalonFX implements IntakeExtendoIO {
  private final TalonFX intakeExtendoMotor;
  // private final MotionMagicVoltage magicPositionVoltage = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  private StatusSignal<Angle> intakeExtendoPosition;
  private StatusSignal<AngularVelocity> intakeExtendoVelocity;
  private StatusSignal<AngularAcceleration> intakeExtendoAcceleration;
  private StatusSignal<Voltage> intakeExtendoAppliedVoltage;
  private StatusSignal<Current> intakeExtendoAppliedCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private Debouncer intakeExtendoConnectedDebounce = new Debouncer(0.1);

  public IntakeExtendoIOTalonFX() {
    intakeExtendoMotor = new TalonFX(Constants.CanId.INTAKE_EXTENDO_CAN_ID, RIO_CAN_BUS);

    var intakeExtendoMotorConfig = new TalonFXConfiguration();

    // PID configuration for velocity mode (Slot 0)
    intakeExtendoMotorConfig.Slot0.kP =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KP;
    intakeExtendoMotorConfig.Slot0.kI =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KI;
    intakeExtendoMotorConfig.Slot0.kD =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KD;
    intakeExtendoMotorConfig.Slot0.kG =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KG;
    intakeExtendoMotorConfig.Slot0.kS =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KS;
    intakeExtendoMotorConfig.Slot0.kV =
            IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KV;

    // PID configuration for position mode (motion magic voltage) (Slot 1)
    intakeExtendoMotorConfig.Slot1.kP =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KP;
    intakeExtendoMotorConfig.Slot1.kI =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KI;
    intakeExtendoMotorConfig.Slot1.kD =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KD;
    intakeExtendoMotorConfig.Slot1.kA =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KA;
    intakeExtendoMotorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    intakeExtendoMotorConfig.Slot1.kG =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KG;
    intakeExtendoMotorConfig.Slot1.kS =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KS;
    intakeExtendoMotorConfig.Slot1.kV =
            IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KV;

    intakeExtendoMotorConfig.MotorOutput.NeutralMode =
        IntakeExtendoTalonFXConstants.MECHANISM_NEUTRAL_MODE;

    intakeExtendoMotorConfig.MotorOutput.Inverted = IntakeExtendoTalonFXConstants.MOTOR_DIRECTION;

    intakeExtendoMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    intakeExtendoMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    intakeExtendoMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    intakeExtendoMotorConfig.CurrentLimits.StatorCurrentLimit =
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps);
    intakeExtendoMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    intakeExtendoMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        linearVelocityToTalonFX(IntakeExtendoTalonFXConstants.MotionMagicConstants.CRUISE_VELOCITY)
            .in(RotationsPerSecond);
    intakeExtendoMotorConfig.MotionMagic.MotionMagicAcceleration =
        linearAccelerationToTalonFX(IntakeExtendoTalonFXConstants.MotionMagicConstants.ACCELERATION)
            .in(RotationsPerSecondPerSecond);
    intakeExtendoMotorConfig.MotionMagic.MotionMagicJerk =
        IntakeExtendoTalonFXConstants.MotionMagicConstants.JERK;

    PhoenixUtil.tryUntilOk(
        5, () -> intakeExtendoMotor.getConfigurator().apply(intakeExtendoMotorConfig, 0.25));

    intakeExtendoPosition = intakeExtendoMotor.getPosition();
    intakeExtendoVelocity = intakeExtendoMotor.getVelocity();
    intakeExtendoAcceleration = intakeExtendoMotor.getAcceleration();
    intakeExtendoAppliedVoltage = intakeExtendoMotor.getMotorVoltage();
    intakeExtendoAppliedCurrent = intakeExtendoMotor.getStatorCurrent();
    currentPidSlot = intakeExtendoMotor.getClosedLoopSlot();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        intakeExtendoPosition,
        intakeExtendoVelocity,
        intakeExtendoAcceleration,
        intakeExtendoAppliedVoltage,
        intakeExtendoAppliedCurrent,
        currentPidSlot);

    ParentDevice.optimizeBusUtilizationForAll(intakeExtendoMotor);
  }
  
  @Override
  public void updateInputs(IntakeExtendoIOInputs inputs) {
    var intakeExtendoSignals =
        BaseStatusSignal.refreshAll(
            intakeExtendoPosition,
            intakeExtendoVelocity,
            intakeExtendoAcceleration,
            intakeExtendoAppliedVoltage,
            intakeExtendoAppliedCurrent,
            currentPidSlot);

    inputs.intakeExtendoMotorConnected =
        intakeExtendoConnectedDebounce.calculate(intakeExtendoSignals.isOK());

    inputs.intakeExtendoDistance = talonFXToExtension(intakeExtendoPosition.getValue());
    inputs.intakeExtendoMotorAngle = intakeExtendoPosition.getValue();

    inputs.intakeExtendoVelocity = intakeExtendoVelocity.getValue();

    inputs.intakeExtendoAcceleration = intakeExtendoAcceleration.getValue();

    inputs.intakeExtendoMotorVolts = intakeExtendoAppliedVoltage.getValue();
    inputs.intakeExtendoMotorCurrent = intakeExtendoAppliedCurrent.getValue();

    inputs.intakeExtendoMotorPIDSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.VELOCITY;
          case 1 -> PIDSlots.POSITION;
          case 2 -> PIDSlots.MOTION_MAGIC_POSITION;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };
  }

  @Override
  public void setIntakeExtendoPosition(Distance position, PIDSlots pidSlot) {
    intakeExtendoMotor.setControl(
        positionVoltage.withPosition(extensionToTalonFX(position)).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setIntakeExtendoPosition(Distance position) {
    setIntakeExtendoPosition(position, PIDSlots.POSITION);
  }

  @Override
  public void setIntakeExtendoVelocity(LinearVelocity velocity, PIDSlots pidSlot) {
    intakeExtendoMotor.setControl(
        velocityVoltage
            .withVelocity(linearVelocityToTalonFX(velocity))
            .withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setIntakeExtendoVelocity(LinearVelocity velocity) {
    setIntakeExtendoVelocity(velocity, PIDSlots.VELOCITY);
  }

  @Override
  public void setIntakeExtendoOpenLoop(double percentOutput) {
    intakeExtendoMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    intakeExtendoMotor.stopMotor();
  }

  /**
   * Converts TalonFX rotations to intake extension. TalonFX reports position in rotations in
   * Phoenix 6. Uses regression formula y = EXTENSION_PER_ROTATION + MIN_EXTENSION where x is
   * rotations and y is meters.
   *
   * @param rotations TalonFX motor rotations
   * @return Extension as a Distance unit
   */
  private Distance talonFXToExtension(Angle rotations) {
    return Meters.of(EXTENSION_PER_ROTATION * rotations.in(Rotations) + MIN_EXTENSION.in(Meters));
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
    return MetersPerSecond.of(rotationsPerSecond.in(RotationsPerSecond) * EXTENSION_PER_ROTATION);
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
    return Rotations.of((extension.minus(MIN_EXTENSION).in(Meters)) / EXTENSION_PER_ROTATION);
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
    return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / EXTENSION_PER_ROTATION);
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
        linearAcceleration.in(MetersPerSecondPerSecond) / EXTENSION_PER_ROTATION);
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
        rotationalAcceleration.in(RotationsPerSecondPerSecond) * EXTENSION_PER_ROTATION);
  }
}
