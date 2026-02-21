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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import java.util.function.Supplier;

public class ShooterShootAtVelocity extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<AngularVelocity> targetVelocity;

  public ShooterShootAtVelocity(
      ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> targetVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetVelocity = targetVelocity;
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
