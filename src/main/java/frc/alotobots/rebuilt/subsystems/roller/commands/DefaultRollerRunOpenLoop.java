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

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultRollerRunOpenLoop extends Command {
  private final RollerSubsystem rollerSubsystem;
  private final DoubleSupplier inputSupplier;

  public DefaultRollerRunOpenLoop(RollerSubsystem rollerSubsystem, DoubleSupplier inputSupplier) {
    this.rollerSubsystem = rollerSubsystem;
    this.inputSupplier = inputSupplier;
    addRequirements(rollerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rollerSubsystem.runRollerPercentOutput(inputSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
