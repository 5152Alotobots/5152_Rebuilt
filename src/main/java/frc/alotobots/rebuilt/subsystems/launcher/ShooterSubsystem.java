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
package frc.alotobots.rebuilt.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.alotobots.rebuilt.subsystems.launcher.constants.ShooterTalonFXConstants.MAX_OPERATOR_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.launcher.io.ShooterIO;
import org.littletonrobotics.junction.Logger;
import frc.alotobots.rebuilt.subsystems.launcher.io.ShooterIOInputsAutoLogged;

public class ShooterSubsystem extends SubsystemBase {

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Controls the shooter to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity = applyVelocityLimitIfNeeded(velocity);
    io.setShooterVelocity(adjustedVelocity);
    Logger.recordOutput("Shooter/ControlType", ShooterIO.PIDSlots.DEFAULT_VELOCITY);
  }

  private AngularVelocity applyVelocityLimitIfNeeded(AngularVelocity velocity) {
    return RadiansPerSecond.of(
        MathUtil.clamp(
            velocity.in(RadiansPerSecond),
            -MAX_OPERATOR_VELOCITY.in(RadiansPerSecond),
            MAX_OPERATOR_VELOCITY.in(RadiansPerSecond)));
  }
}
