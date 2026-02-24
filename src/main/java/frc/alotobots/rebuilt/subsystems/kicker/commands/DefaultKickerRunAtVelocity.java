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
package frc.alotobots.rebuilt.subsystems.kicker.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import java.util.function.Supplier;

public class DefaultKickerRunAtVelocity extends Command {
  private final KickerSubsystem kickerSubsystem;
  private final Supplier<AngularVelocity> velocitySupplier;

  public DefaultKickerRunAtVelocity(
      KickerSubsystem kickerSubsystem, Supplier<AngularVelocity> velocitySupplier) {
    this.kickerSubsystem = kickerSubsystem;
    this.velocitySupplier = velocitySupplier;
    addRequirements(kickerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    kickerSubsystem.runToTargetVelocity(velocitySupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
