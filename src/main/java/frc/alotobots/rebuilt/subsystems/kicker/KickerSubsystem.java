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
package frc.alotobots.rebuilt.subsystems.kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterTalonFXConstants.MAX_OPERATOR_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIO;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {

  private KickerIO io;
  private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Controls the kicker to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runKickerToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity = applyVelocityLimitIfNeeded(velocity);
    io.setKickerVelocity(adjustedVelocity);
    Logger.recordOutput("Hopper/ControlType", KickerIO.PIDSlots.DEFAULT_VELOCITY);
  }

  // todo fix io method name
  public void runKickerPercentOutput(double percentOutput) {
    io.setKickerOpenLoop(percentOutput);
  }

  private AngularVelocity applyVelocityLimitIfNeeded(AngularVelocity velocity) {
    return RadiansPerSecond.of(
        MathUtil.clamp(
            velocity.in(RadiansPerSecond),
            -MAX_OPERATOR_VELOCITY.in(RadiansPerSecond),
            MAX_OPERATOR_VELOCITY.in(RadiansPerSecond)));
  }

  public void stop() {
    io.stop();
  }
}
