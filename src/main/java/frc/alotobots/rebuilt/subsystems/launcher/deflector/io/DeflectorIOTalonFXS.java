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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.alotobots.Constants.CanId.DEFAULT_CAN_FREQUENCY;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorTalonFXSConstants.DEFLECTOR_ROTATION_PER_ROTATION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.alotobots.Constants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorTalonFXSConstants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorTalonFXSConstants.PIDConstants;
import frc.alotobots.util.PhoenixUtil;
import jdk.jshell.spi.ExecutionControl.NotImplementedException;

import org.littletonrobotics.junction.Logger;

public class DeflectorIOTalonFXS implements DeflectorIO {
  private final TalonFXS deflectorMotor;
  private final CANcoder deflectorEncoder;

  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  // private final DigitalInput backLimitSwitch;
  // private final Debouncer backLimitDebouncer;
  private final Debouncer deflectorMotorConnectedDebouncer;
  private final Debouncer deflectorEncoderConnectedDebouncer;

  private StatusSignal<Angle> deflectorMotorPosition;
  private StatusSignal<AngularVelocity> deflectorMotorVelocity;
  private StatusSignal<AngularAcceleration> deflectorMotorAcceleration;
  private StatusSignal<Voltage> deflectorMotorVoltage;
  private StatusSignal<Current> deflectorMotorCurrent;
  private StatusSignal<Integer> currentPidSlot;
  private StatusSignal<ControlModeValue> deflectorMotorControlMode;

  public DeflectorIOTalonFXS() {
    deflectorMotor = new TalonFXS(Constants.CanId.DEFLECTOR_MOTOR_CAN_ID, RIO_CAN_BUS);
    deflectorEncoder = new CANcoder(Constants.CanId.DEFLECTOR_ENCODER_CAN_ID, RIO_CAN_BUS);
    deflectorMotorConnectedDebouncer = new Debouncer(0.1);
    deflectorEncoderConnectedDebouncer = new Debouncer(0.1);
    // backLimitDebouncer = new Debouncer(0.1);
    // backLimitSwitch = new DigitalInput(0);

    var deflectorMotorConfig = new TalonFXSConfiguration();
    deflectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deflectorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Set feed back source to CANcoder
    deflectorMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    deflectorMotorConfig.ExternalFeedback.withRemoteCANcoder(deflectorEncoder);
    
    // Configure controler output for the Minion
    deflectorMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    deflectorMotorConfig.Slot0.kP = PIDConstants.PositionPIDConstants.DEFLECTOR_POSITION_KP;
    deflectorMotorConfig.Slot0.kI = PIDConstants.PositionPIDConstants.DEFLECTOR_POSITION_KI;
    deflectorMotorConfig.Slot0.kD = PIDConstants.PositionPIDConstants.DEFLECTOR_POSITION_KD;
    deflectorMotorConfig.Slot0.kG = PIDConstants.PositionPIDConstants.DEFLECTOR_POSITION_KG;

    PhoenixUtil.tryUntilOk(
        5, () -> deflectorMotor.getConfigurator().apply(deflectorMotorConfig, 0.25));

    deflectorMotorPosition = deflectorMotor.getPosition();
    deflectorMotorVelocity = deflectorMotor.getVelocity();
    deflectorMotorAcceleration = deflectorMotor.getAcceleration();
    deflectorMotorVoltage = deflectorMotor.getMotorVoltage();
    deflectorMotorCurrent = deflectorMotor.getStatorCurrent();
    currentPidSlot = deflectorMotor.getClosedLoopSlot();
    deflectorMotorControlMode = deflectorMotor.getControlMode();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DEFAULT_CAN_FREQUENCY,
        deflectorMotorPosition,
        deflectorMotorVelocity,
        deflectorMotorAcceleration,
        deflectorMotorVoltage,
        deflectorMotorCurrent,
        currentPidSlot,
        deflectorMotorControlMode);

    ParentDevice.optimizeBusUtilizationForAll(deflectorMotor);
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    var motorSignals =
        BaseStatusSignal.refreshAll(
            deflectorMotorPosition,
            deflectorMotorVelocity,
            deflectorMotorAcceleration,
            deflectorMotorVoltage,
            deflectorMotorCurrent,
            currentPidSlot,
            deflectorMotorControlMode);

    inputs.deflectorMotorPidSlot =
        switch (currentPidSlot.getValue()) {
          case 0 -> PIDSlots.DEFAULT_POSITION;
          // case 1 -> PIDSlots.VELOCITY;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + currentPidSlot.getValue());
        };

    inputs.deflectorMotorConnected =
        deflectorMotorConnectedDebouncer.calculate(motorSignals.isOK());
    inputs.deflectorMotorVelocity = deflectorMotorVelocity.getValue();
    inputs.deflectorMotorAcceleration = deflectorMotorAcceleration.getValue();
    inputs.deflectorMotorVolts = deflectorMotorVoltage.getValue();
    inputs.deflectorMotorCurrent = deflectorMotorCurrent.getValue();
    inputs.deflectorMotorAngle = deflectorMotorPosition.getValue();
  }

  @Override
  public void setDeflectorPosition(Angle position) {
    setDeflectorPosition(deflectorAngleToTalonFXS(position), PIDSlots.DEFAULT_POSITION);
  }

  /**
   * * Commands the deflector to move to a specified position using closed-loop control.
   *
   * @param position The target position for the deflector
   * @param pidSlot The PID slot to use for the control
   */
  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    deflectorMotor.setControl(
        positionControl.withPosition(deflectorAngleToTalonFXS(position)).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    // IF IMPLEMENTED NEED TO CONVERT FROM HOOD VELOCITY TO MOTOR 
    // deflectorMotor.setControl(velocityControl.withVelocity(velocity).withSlot(pidSlot.ordinal()));
  }

  @Override
  public void setDeflectorVelocity(AngularVelocity velocity) {
    // IF IMPLEMENTED NEED TO CONVERT FROM HOOD VELOCITY TO MOTOR 
    // deflectorMotor.setControl(
        // velocityControl.withVelocity(velocity).withSlot(PIDSlots.VELOCITY.ordinal()));
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

  /**
   * Converts TalonFXS motor position to deflector angle. Uses regression formula y = SLOPE * x +
   * MAX_ANGLE where x is motor position in radians and y is deflector angle in radians.
   *
   * @param motorPosition TalonFXS motor position as an Angle unit
   * @return Deflector angle as an Angle unit
   */
  private Angle talonFXSToDeflectorAngle(Angle motorPosition) {
    return Radians.of(
        DEFLECTOR_ROTATION_PER_ROTATION * motorPosition.in(Radians)
            + DEFLECTOR_MAX_ANGLE.in(Radians));
  }

  /**
   * Converts TalonFXS motor velocity to deflector angular velocity. Uses the slope from the
   * regression formula y = SLOPE * x + MAX_ANGLE as the conversion factor.
   *
   * @param motorVelocity TalonFXS motor rotational velocity as an AngularVelocity unit
   * @return Deflector angular velocity as an AngularVelocity unit
   */
  private AngularVelocity talonFXSToDeflectorAngularVelocity(AngularVelocity motorVelocity) {
    return RadiansPerSecond.of(
        motorVelocity.in(RadiansPerSecond) * DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angle to TalonFXS motor position. Uses inverse of regression formula y = SLOPE
   * * x + MAX_ANGLE, solving for x: x = (y - MAX_ANGLE) / SLOPE where y is deflector angle in
   * radians and x is motor position in radians.
   *
   * @param deflectorAngle Deflector angle as an Angle unit
   * @return TalonFXS motor position as an Angle unit
   */
  private Angle deflectorAngleToTalonFXS(Angle deflectorAngle) {
    return Radians.of(
        (deflectorAngle.in(Radians) - DEFLECTOR_MAX_ANGLE.in(Radians))
            / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angular velocity to TalonFXS motor velocity. Uses inverse slope from
   * regression formula y = SLOPE * x + MAX_ANGLE.
   *
   * @param deflectorAngularVelocity Deflector angular velocity as an AngularVelocity unit
   * @return TalonFXS motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity deflectorAngularVelocityToTalonFXS(
      AngularVelocity deflectorAngularVelocity) {
    return RadiansPerSecond.of(
        deflectorAngularVelocity.in(RadiansPerSecond) / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts deflector angular acceleration to TalonFXS motor rotational acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param deflectorAngularAcceleration Deflector angular acceleration as an AngularAcceleration
   *     unit
   * @return TalonFXS motor rotational acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration deflectorAngularAccelerationToTalonFXS(
      AngularAcceleration deflectorAngularAcceleration) {
    return RadiansPerSecondPerSecond.of(
        deflectorAngularAcceleration.in(RadiansPerSecondPerSecond)
            / DEFLECTOR_ROTATION_PER_ROTATION);
  }

  /**
   * Converts TalonFXS motor rotational acceleration to deflector angular acceleration. Uses the same
   * conversion factor as velocity since acceleration is the time derivative of velocity.
   *
   * @param motorAcceleration TalonFXS motor rotational acceleration as an AngularAcceleration unit
   * @return Deflector angular acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration vortexToDeflectorAngularAcceleration(
      AngularAcceleration motorAcceleration) {
    return RadiansPerSecondPerSecond.of(
        motorAcceleration.in(RadiansPerSecondPerSecond) * DEFLECTOR_ROTATION_PER_ROTATION);
  }

}
