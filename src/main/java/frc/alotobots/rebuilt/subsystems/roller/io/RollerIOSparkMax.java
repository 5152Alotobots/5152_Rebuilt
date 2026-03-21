/*
 * ALOTOBOTS - FRC Team 5152
 * https://github.com/5152Alotobots
 * Copyright (C) 2026 ALOTOBOTS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Source code must be publicly available on GitHub or an alternative web accessible site
 */
package frc.alotobots.rebuilt.subsystems.roller.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.roller.constants.RollerSparkMaxConstants;
import frc.alotobots.Constants;

public class RollerIOSparkMax implements RollerIO {
  private final SparkMax rollerMotor;
  private final Debouncer rollerMotorConnectedDebouncer;

  public RollerIOSparkMax() {
    rollerMotor = new SparkMax(Constants.CanId.ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);
    rollerMotorConnectedDebouncer = new Debouncer(0.1);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

    rollerMotorConfig.idleMode(RollerSparkMaxConstants.ROLLER_MECHANISM_NEUTRAL_MODE);
    rollerMotorConfig.inverted(RollerSparkMaxConstants.ROLLER_MOTOR_DIRECTION_INVERTED);
    rollerMotorConfig.smartCurrentLimit(
        (int) RollerSparkMaxConstants.MotorSafetyLimits.ROLLER_TORQUE_AMP_LIMIT.in(Amps));

    // Velocity (Slot 0 - Default)
    rollerMotorConfig.closedLoop.p(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KP,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.i(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KI,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.d(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KD,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.feedForward.kG(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KG,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.feedForward.kS(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KS,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.feedForward.kV(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_KV,
        ClosedLoopSlot.kSlot0);
    rollerMotorConfig.closedLoop.allowedClosedLoopError(
        RollerSparkMaxConstants.PIDConstants.VelocityPIDConstants.ROLLER_VELOCITY_ALLOWED_CLOSED_LOOP_ERROR,
        ClosedLoopSlot.kSlot0);

    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerMotorConnected =
        rollerMotorConnectedDebouncer.calculate(rollerMotor.getBusVoltage() > 0);

    inputs.rollerMotorVelocity =
        RotationsPerSecond.of(rollerMotor.getEncoder().getVelocity());
        
    // No roller motor acceleration because REV is shit
    
    inputs.rollerMotorVolts =
        Volts.of(rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage());
    inputs.rollerMotorCurrent = Amps.of(rollerMotor.getOutputCurrent());
  }

  @Override
  public void setRollerVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    ClosedLoopSlot slot =
        switch (pidSlot) {
          case DEFAULT_VELOCITY -> ClosedLoopSlot.kSlot0;
          default -> throw new IllegalArgumentException(
              "No defined PID slot for value: " + pidSlot.ordinal());
        };


    rollerMotor
        .getClosedLoopController()
        .setSetpoint(
            velocity.in(RotationsPerSecond),
            ControlType.kVelocity,
            slot);
  }

  @Override
  public void setRollerVelocity(AngularVelocity velocity) {
    setRollerVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setRollerOpenLoop(double percentOutput) {
    Logger.recordOutput("Roller/openLoopPercentOut", percentOutput);
    rollerMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }
}
