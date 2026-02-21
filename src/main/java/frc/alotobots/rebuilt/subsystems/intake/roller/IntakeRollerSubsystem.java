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
package frc.alotobots.rebuilt.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIO;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants.Limits.LIMITS_ENABLED;

public class IntakeRollerSubsystem extends SubsystemBase {

  private IntakeRollerIO io;
  private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRollerSubsystem(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);
  }

  public void runAtPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(
            percentOutput,
            -IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE,
            IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE);
    io.setIntakeRollerOpenLoop(LIMITS_ENABLED ? adjustedOutput : percentOutput);
    Logger.recordOutput("Intake/Roller/ControlType", "PERCENT_OUTPUT");
  }

  public void stop() {
    io.stop();
  }
}
