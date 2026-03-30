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
package frc.alotobots.rebuilt.subsystems.roller.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import java.util.function.Supplier;

public class DefaultRollerRunAtVelocity extends Command {
  private final BeltSubsystem beltSubsystem;
  private final Supplier<AngularVelocity> velocitySupplier;

  public DefaultRollerRunAtVelocity(
      BeltSubsystem beltSubsystem, Supplier<AngularVelocity> velocitySupplier) {
    this.beltSubsystem = beltSubsystem;
    this.velocitySupplier = velocitySupplier;
    addRequirements(beltSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    beltSubsystem.runBeltToTargetVelocity(velocitySupplier.get());
  }

  @Override
  public void end(boolean interrupted) {
    beltSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
