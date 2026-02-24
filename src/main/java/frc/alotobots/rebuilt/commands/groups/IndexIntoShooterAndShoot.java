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
package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants;

/**
 * Command group that spins up the shooter and then feeds a game piece into it.
 *
 * <p>Starts the shooter flywheel at the configured test velocity and uses it as a deadline command.
 * Once the shooter reports that it has reached its target velocity, the belt and kicker are run
 * simultaneously to index the game piece into the shooter.
 */
public class IndexIntoShooterAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new IndexIntoShooterAndShoot command group.
   *
   * @param beltSubsystem The belt subsystem used to move the game piece toward the shooter
   * @param kickerSubsystem The kicker subsystem used to feed the game piece into the shooter
   * @param shooterSubsystem The shooter subsystem used to launch the game piece
   */
  public IndexIntoShooterAndShoot(
      BeltSubsystem beltSubsystem,
      KickerSubsystem kickerSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addCommands(
    new DefaultShooterRunAtVelocity(
            shooterSubsystem, () -> ShooterConstants.Setpoints.SHOOTER_TEST_VELOCITY)
        .deadlineFor(
            new SequentialCommandGroup(
                // Wait for shooter to spin up
                new WaitUntilCommand(shooterSubsystem::isAtTargetVelocity),
                // Then feed
                new ParallelCommandGroup(
                    new DefaultBeltRunAtVelocity(
                        beltSubsystem, () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                    new DefaultKickerRunAtVelocity(
                        kickerSubsystem,
                        () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY))))
    );
  }
}
