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
import static frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants.Limits.*;

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
    Logger.processInputs("Kicker", inputs);
  }

  /**
   * Controls the kicker to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -MAX_SPEED.in(RadiansPerSecond),
                MAX_SPEED.in(RadiansPerSecond)));
    io.setKickerVelocity(LIMITS_ENABLED ? adjustedVelocity : velocity);
    Logger.recordOutput("Kicker/ControlType", KickerIO.PIDSlots.DEFAULT_VELOCITY);
  }

  public void runAtPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setKickerOpenLoop(LIMITS_ENABLED ? adjustedOutput : percentOutput);
    Logger.recordOutput("Kicker/ControlType", "PERCENT_OUTPUT");
  }

  public void stop() {
    io.stop();
  }
}
