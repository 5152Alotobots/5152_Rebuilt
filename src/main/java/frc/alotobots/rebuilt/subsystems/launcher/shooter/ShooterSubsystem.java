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
package frc.alotobots.rebuilt.subsystems.launcher.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.MAX_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIO;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher/Shooter", inputs);
  }

  /**
   * Controls the shooter to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_SPEED and MAX_SPEED
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -MAX_SPEED.in(RadiansPerSecond),
                MAX_SPEED.in(RadiansPerSecond)));
    io.setShooterVelocity(adjustedVelocity);
    Logger.recordOutput("Launcher/Shooter/ControlType", ShooterIO.PIDSlots.DEFAULT_VELOCITY);
  }

  public void runShooterPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setShooterOpenLoop(percentOutput);
    Logger.recordOutput("Launcher/Shooter/ControlType", "PERCENT_OUTPUT");
  }

  public void stop() {
    io.stop();
  }
}
