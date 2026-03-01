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
package frc.alotobots.rebuilt.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.climber.ClimberSubsystem;
import java.util.function.DoubleSupplier;

public class ClimberRunOpenLoop extends Command {
  private final ClimberSubsystem climberSubsystem;
  private final DoubleSupplier input;

  public ClimberRunOpenLoop(ClimberSubsystem climberSubsystem, DoubleSupplier input) {
    this.climberSubsystem = climberSubsystem;
    this.input = input;

    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {
    climberSubsystem.setClimberOpenLoop(input.getAsDouble());
  }
}
