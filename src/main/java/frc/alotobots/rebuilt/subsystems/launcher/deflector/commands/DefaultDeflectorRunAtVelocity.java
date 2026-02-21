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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.commands;

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.MAX_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDeflectorRunAtVelocity extends Command {
  private final DeflectorSubsystem deflectorSubsystem;
  private final DoubleSupplier input;

  public DefaultDeflectorRunAtVelocity(
      DeflectorSubsystem deflectorSubsystem, DoubleSupplier input) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.input = input;

    addRequirements(deflectorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double adjustedInput = MathUtil.clamp(input.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT);
    AngularVelocity velocity = MAX_SPEED.times(adjustedInput);
    deflectorSubsystem.runToTargetVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    deflectorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
