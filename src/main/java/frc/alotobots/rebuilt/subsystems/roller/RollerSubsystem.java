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
package frc.alotobots.rebuilt.subsystems.roller;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.alotobots.rebuilt.subsystems.roller.constants.RollerConstants.Limits.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.roller.io.RollerIO;
import frc.alotobots.rebuilt.subsystems.roller.io.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {

  private RollerIO io;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  /**
   * Controls the roller to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in radians per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runRollerToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        RadiansPerSecond.of(
            MathUtil.clamp(
                velocity.in(RadiansPerSecond),
                -MAX_SPEED.in(RadiansPerSecond),
                MAX_SPEED.in(RadiansPerSecond)));
    io.setRollerVelocity(LIMITS_ENABLED ? adjustedVelocity : velocity);
    Logger.recordOutput("Roller/ControlType", RollerIO.PIDSlots.DEFAULT_VELOCITY);
  }

  public void runRollerPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setRollerOpenLoop(LIMITS_ENABLED ? adjustedOutput : percentOutput);
    Logger.recordOutput("Roller/ControlType", "PERCENT_OUTPUT");
  }

  public void stop() {
    io.stop();
  }
}
