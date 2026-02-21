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
package frc.alotobots.rebuilt.subsystems.launcher.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
  private static final double GEARING = 1.0;
  private static final double MOMENT_OF_INERTIA = 0.004; // kg * m^2
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);

  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(GEARBOX, MOMENT_OF_INERTIA, GEARING),
          GEARBOX,
          GEARING);

  @AutoLogOutput private double appliedVolts = 0.0;
  private boolean isClosedLoop = false;
  private PIDSlots currentPidSlot = PIDSlots.DEFAULT_VELOCITY;

  // Controllers for Sim (Simple representation of on-motor PID)
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0.1, 0.001); // kS, kV
  private final PIDController pid = new PIDController(0.05, 0.0, 0.0); // kP, kI, kD

  // Acceleration Calculation State
  private double previousVelocityRadPerSec = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // 1. Update Physics
    // We update the sim with the voltage applied to the *system*.
    // Since motors are parallel, they get the same voltage.
    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);

    // 2. Calculate Kinematics (Shared by both motors)
    double currentVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    double currentAccelRadPerSec2 = (currentVelocityRadPerSec - previousVelocityRadPerSec) / 0.02;

    // Save for next loop
    previousVelocityRadPerSec = currentVelocityRadPerSec;

    // 3. Populate Inputs
    inputs.shooterMotorLeftPidSlot = currentPidSlot;
    inputs.shooterMotorRightPidSlot = currentPidSlot;

    inputs.shooterMotorLeftConnected = true;
    inputs.shooterMotorRightConnected = true;

    // Both motors spin at the exact same speed
    inputs.shooterMotorLeftVelocity = RadiansPerSecond.of(currentVelocityRadPerSec);
    inputs.shooterMotorRightVelocity = RadiansPerSecond.of(currentVelocityRadPerSec);

    inputs.shooterMotorLeftAcceleration = RadiansPerSecondPerSecond.of(currentAccelRadPerSec2);
    inputs.shooterMotorRightAcceleration = RadiansPerSecondPerSecond.of(currentAccelRadPerSec2);

    inputs.shooterMotorLeftVolts = Volts.of(appliedVolts);
    inputs.shooterMotorRightVolts = Volts.of(appliedVolts);

    // Current is total draw of the gearbox divided by 2 motors
    double currentPerMotor = flywheelSim.getCurrentDrawAmps() / 2.0;
    inputs.shooterMotorLeftCurrent = Amps.of(currentPerMotor);
    inputs.shooterMotorRightCurrent = Amps.of(currentPerMotor);
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    if (velocity == null) throw new IllegalArgumentException("Velocity cannot be null");
    if (pidSlot == null) throw new IllegalArgumentException("PID Slot cannot be null");

    // Convert target to Rad/Sec
    double targetRadPerSec = velocity.in(RadiansPerSecond);
    Logger.recordOutput("Shooter/SetpointRPM", velocity.in(RotationsPerSecond) * 60.0);

    currentPidSlot = pidSlot;
    isClosedLoop = true;

    // Calculate simulated output
    // In a real Sim implementation, you might want to tune the feedforward/PID constants
    // declared at the top of the class to make the "spin up" time match your real robot.
    double ffVolts = feedforward.calculate(targetRadPerSec);
    double pidVolts = pid.calculate(flywheelSim.getAngularVelocityRadPerSec(), targetRadPerSec);

    appliedVolts = ffVolts + pidVolts;
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    setShooterVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setShooterOpenLoop(double percentOutput) {
    Logger.recordOutput("Shooter/OpenLoopPercent", percentOutput);
    isClosedLoop = false;
    appliedVolts = percentOutput * 12.0;
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    appliedVolts = 0.0;
  }
}
