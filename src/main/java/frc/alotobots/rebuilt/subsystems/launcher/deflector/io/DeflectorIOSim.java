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

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class DeflectorIOSim implements DeflectorIO {
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60(1);
  private static final double GEARING = 100.0;
  private static final double MOI_KG_M2 = 0.01;
  private static final double ARM_LENGTH_METERS = 0.2;
  private static final double MIN_ANGLE_RADS = 0.0;
  private static final double MAX_ANGLE_RADS = Math.PI / 2.0;
  private static final boolean SIMULATE_GRAVITY = true;

  private final SingleJointedArmSim deflectorSim =
      new SingleJointedArmSim(
          MOTOR_MODEL,
          GEARING,
          MOI_KG_M2,
          ARM_LENGTH_METERS,
          MIN_ANGLE_RADS,
          MAX_ANGLE_RADS,
          SIMULATE_GRAVITY,
          MIN_ANGLE_RADS);

  private final PIDController pid = new PIDController(10.0, 0.0, 0.0);

  @AutoLogOutput private double appliedVolts = 0.0;
  private boolean isClosedLoop = false;
  private double targetAngleRads = 0.0;
  private ControlModeValue currentControlMode = ControlModeValue.DisabledOutput;
  private PIDSlots currentPidSlot = PIDSlots.DEFAULT_POSITION;
  private double previousVelocityRadPerSec = 0.0;

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    if (isClosedLoop) {
      double pidVolts = pid.calculate(deflectorSim.getAngleRads(), targetAngleRads);
      double kG = 0.5;
      double feedforwardVolts = kG * Math.cos(deflectorSim.getAngleRads());
      appliedVolts = pidVolts + feedforwardVolts;
    }

    deflectorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    deflectorSim.update(0.02);

    double currentVelocity = deflectorSim.getVelocityRadPerSec();
    double currentAccel = (currentVelocity - previousVelocityRadPerSec) / 0.02;
    previousVelocityRadPerSec = currentVelocity;

    inputs.deflectorMotorConnected = true;
    inputs.deflectorMotorPidSlot = currentPidSlot;
    inputs.deflectorMotorAngle = Radian.of(deflectorSim.getAngleRads());
    inputs.deflectorMotorVelocity = RadiansPerSecond.of(currentVelocity);
    inputs.deflectorMotorAcceleration = RadiansPerSecondPerSecond.of(currentAccel);
    inputs.deflectorMotorVolts = Volts.of(appliedVolts);
    inputs.deflectorMotorCurrent = Amps.of(deflectorSim.getCurrentDrawAmps());
    inputs.backLimit = deflectorSim.hasHitLowerLimit();
  }

  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    if (position == null) throw new IllegalArgumentException("Position cannot be null");
    targetAngleRads = position.in(Radians);
    currentPidSlot = pidSlot;
    isClosedLoop = true;
    currentControlMode = ControlModeValue.PositionVoltage;
  }

  @Override
  public void setDeflectorPosition(Angle position) {
    setDeflectorPosition(position, PIDSlots.DEFAULT_POSITION);
  }

  @Override
  public void setDeflectorOpenLoop(double percentOutput) {
    isClosedLoop = false;
    currentControlMode = ControlModeValue.DutyCycleOut;
    appliedVolts = percentOutput * 12.0;
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    currentControlMode = ControlModeValue.DisabledOutput;
    appliedVolts = 0.0;
  }
}
