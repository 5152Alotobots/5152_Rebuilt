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
package frc.alotobots.rebuilt.subsystems.launcher.shooter.commands;

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants.Limits.SHOOTER_MAX_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DefaultShooterRunAtVelocity extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<AngularVelocity> targetVelocity;

  public DefaultShooterRunAtVelocity(
      ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> targetVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetVelocity = targetVelocity;

    addRequirements(shooterSubsystem);
  }

  public DefaultShooterRunAtVelocity(
      ShooterSubsystem shooterSubsystem, DoubleSupplier controllerInput) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetVelocity =
        () ->
            SHOOTER_MAX_VELOCITY.times(
                MathUtil.clamp(controllerInput.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT));

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.runToTargetVelocity(targetVelocity.get());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
