/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.turret.TurretSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TurretDefault extends Command {
  private final TurretSubsystem turretSubsystem;
  private final DoubleSupplier input;

  public TurretDefault(TurretSubsystem turretSubsystem, DoubleSupplier input) {
    // No requirements, runs when no other commands are running

    this.turretSubsystem = turretSubsystem;
    this.input = input;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.runAtPercentOutput(input.getAsDouble());
    Logger.recordOutput("turretDef/isRunning", "Going" + input.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false; // Never finishes on its own
  }
}
