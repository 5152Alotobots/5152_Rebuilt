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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorRunToPosition;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.constants.ShooterConstants;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretRunToPosition;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretConstants;

/**
 * Command group that spins up the shooter and then feeds a game piece into it.
 *
 * <p>Starts the shooter flywheel at the configured test velocity and uses it as a deadline command.
 * Once the shooter reports that it has reached its target velocity, the belt and kicker are run
 * simultaneously to index the game piece into the shooter.
 */
public class LauncherTargetHubFixedAndShoot extends SequentialCommandGroup {

  public record FixedShootingPosition(
      Angle deflectorAngle, AngularVelocity shooterVelocity, Angle turretAngle) {}

  public static final FixedShootingPosition FIXED_SHOOTING_POSITION_LEFT_TRENCH =
      new FixedShootingPosition(
          DeflectorConstants.Setpoints.Fixed.FIXED_DEFLECTOR_ANGLE_LEFT_TRENCH,
          ShooterConstants.Setpoints.Fixed.FIXED_SHOOTER_VELOCITY_LEFT_TRENCH,
          TurretConstants.Setpoints.Fixed.FIXED_TURRET_ANGLE_LEFT_TRENCH);
  public static final FixedShootingPosition FIXED_SHOOTING_POSITION_CENTER =
      new FixedShootingPosition(
          DeflectorConstants.Setpoints.Fixed.FIXED_DEFLECTOR_ANGLE_CENTER,
          ShooterConstants.Setpoints.Fixed.FIXED_SHOOTER_VELOCITY_CENTER,
          TurretConstants.Setpoints.Fixed.FIXED_TURRET_ANGLE_CENTER);
  public static final FixedShootingPosition FIXED_SHOOTING_POSITION_RIGHT_TRENCH =
      new FixedShootingPosition(
          DeflectorConstants.Setpoints.Fixed.FIXED_DEFLECTOR_ANGLE_RIGHT_TRENCH,
          ShooterConstants.Setpoints.Fixed.FIXED_SHOOTER_VELOCITY_RIGHT_TRENCH,
          TurretConstants.Setpoints.Fixed.FIXED_TURRET_ANGLE_RIGHT_TRENCH);

  /**
   * Creates a new LauncherTargetHubFixedAndShoot command group.
   *
   * @param beltSubsystem The belt subsystem used to move the game piece toward the shooter
   * @param kickerSubsystem The kicker subsystem used to feed the game piece into the shooter
   * @param shooterSubsystem The shooter subsystem used to launch the game piece
   */
  public LauncherTargetHubFixedAndShoot(
      BeltSubsystem beltSubsystem,
      KickerSubsystem kickerSubsystem,
      ShooterSubsystem shooterSubsystem,
      DeflectorSubsystem deflectorSubsystem,
      TurretSubsystem turretSubsystem,
      FixedShootingPosition position) {
    addCommands(
        new ParallelCommandGroup(
                new DeflectorRunToPosition(deflectorSubsystem, position.deflectorAngle),
                new TurretRunToPosition(turretSubsystem, position.turretAngle),
                new DefaultShooterRunAtVelocity(shooterSubsystem, position::shooterVelocity))
            .deadlineFor(
                new SequentialCommandGroup(
                    // Wait for shooter to spin up
                    new WaitUntilCommand(shooterSubsystem::isAtTargetVelocity),
                    // Then feed
                    new ParallelCommandGroup(
                        new DefaultBeltRunAtVelocity(
                            beltSubsystem,
                            () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                        new DefaultKickerRunAtVelocity(
                            kickerSubsystem,
                            () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY)))));
  }
}
