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
package frc.alotobots.rebuilt.commands.manual;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import java.util.function.Supplier;

public class KickerShooterManual extends SequentialCommandGroup {
  public KickerShooterManual(
      ShooterSubsystem shooterSubsystem,
      KickerSubsystem kickerSubsystem,
      BeltSubsystem beltSubsystem,
      Supplier<AngularVelocity> velocity,
      Trigger launchButton) {
    addCommands(
        new InstantCommand(() -> shooterSubsystem.runShooterPercentOutput(-1)),
        new InstantCommand(() -> kickerSubsystem.runKickerPercentOutput(1)),
        new WaitUntilCommand(launchButton),
        new InstantCommand(() -> beltSubsystem.runBeltPercentOutput(.5)),
        new WaitCommand(1),
        new WaitCommand(10).raceWith(new WaitUntilCommand(launchButton)),
        new InstantCommand(shooterSubsystem::stop),
        new InstantCommand(beltSubsystem::stop),
        new InstantCommand(kickerSubsystem::stop));
  }
}
