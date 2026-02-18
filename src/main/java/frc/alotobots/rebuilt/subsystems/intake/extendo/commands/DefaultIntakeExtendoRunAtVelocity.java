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
package frc.alotobots.rebuilt.subsystems.intake.extendo.commands;

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Limits.MAX_OPERATOR_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultIntakeExtendoRunAtVelocity extends Command {

  private final IntakeExtendoSubsystem intakeExtendoSubsystem;
  private final DoubleSupplier input;

  public DefaultIntakeExtendoRunAtVelocity(
      IntakeExtendoSubsystem intakeExtendoSubsystem, DoubleSupplier input) {
    this.intakeExtendoSubsystem = intakeExtendoSubsystem;
    this.input = input;
    addRequirements(intakeExtendoSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double adjustedInput = MathUtil.clamp(input.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT);
    LinearVelocity velocity = MAX_OPERATOR_VELOCITY.times(adjustedInput);
    intakeExtendoSubsystem.runToTargetVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    intakeExtendoSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
