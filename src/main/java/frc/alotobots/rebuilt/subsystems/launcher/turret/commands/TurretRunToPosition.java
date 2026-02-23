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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;

/**
 * Command that rotates the turret to a fixed angle and finishes once it arrives.
 *
 * <p>The target angle is set once during {@link #initialize()} and the command completes when
 * {@link TurretSubsystem#isAtTargetAngle()} returns {@code true}, indicating the turret has been
 * within the position tolerance for the required debounce duration. The turret is stopped if the
 * command is interrupted before reaching the target.
 */
public class TurretRunToPosition extends Command {
  private final TurretSubsystem turretSubsystem;
  private final Angle targetAngle;

  /**
   * Creates a new TurretRunToPosition command.
   *
   * @param turretSubsystem The turret subsystem this command will control
   * @param targetAngle The fixed angle to rotate the turret to
   */
  public TurretRunToPosition(TurretSubsystem turretSubsystem, Angle targetAngle) {
    this.turretSubsystem = turretSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(turretSubsystem);
  }

  /**
   * Commands the turret to begin moving to the target angle.
   */
  @Override
  public void initialize() {
    turretSubsystem.runToTargetAngle(targetAngle);
  }

  /** No per-loop logic required; the hardware controller handles movement. */
  @Override
  public void execute() {}

  /**
   * Stops the turret when the command ends.
   *
   * @param interrupted Whether the command was interrupted before reaching the target
   */
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  /**
   * Returns {@code true} once the turret has stably reached its target angle.
   *
   * @return Whether the turret is at the target angle
   */
  @Override
  public boolean isFinished() {
    return turretSubsystem.isAtTargetAngle();
  }
}
