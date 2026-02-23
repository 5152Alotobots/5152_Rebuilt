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
package frc.alotobots.rebuilt.subsystems.launcher.turret.commands;

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants.Limits.TURRET_MAX_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DefaultTurretRunAtVelocity extends Command {
  private final TurretSubsystem turretSubsystem;
  private final Supplier<AngularVelocity> targetVelocity;

  public DefaultTurretRunAtVelocity(
      TurretSubsystem turretSubsystem, Supplier<AngularVelocity> targetVelocity) {
    this.turretSubsystem = turretSubsystem;
    this.targetVelocity = targetVelocity;

    addRequirements(turretSubsystem);
  }

  public DefaultTurretRunAtVelocity(
      TurretSubsystem turretSubsystem, DoubleSupplier controllerInput) {
    this.turretSubsystem = turretSubsystem;
    this.targetVelocity =
        () ->
            TURRET_MAX_VELOCITY.times(
                MathUtil.clamp(controllerInput.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT));

    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.runToTargetVelocity(targetVelocity.get());
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
