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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;

/**
 * Command that moves the deflector to a fixed angle and finishes once it arrives.
 *
 * <p>The target angle is set once during {@link #initialize()} and the command completes when
 * {@link DeflectorSubsystem#isAtTargetAngle()} returns {@code true}, indicating the deflector has
 * been within the position tolerance for the required debounce duration. The deflector is stopped
 * if the command is interrupted before reaching the target.
 */
public class DeflectorRunToPosition extends Command {
  private final DeflectorSubsystem deflectorSubsystem;
  private final Angle angle;

  /**
   * Creates a new DeflectorRunToPosition command.
   *
   * @param deflectorSubsystem The deflector subsystem this command will control
   * @param angle The target angle to move the deflector to
   */
  public DeflectorRunToPosition(DeflectorSubsystem deflectorSubsystem, Angle angle) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.angle = angle;

    addRequirements(deflectorSubsystem);
  }

  /** Commands the deflector to begin moving to the target angle. */
  @Override
  public void initialize() {
    deflectorSubsystem.runToTargetAngle(angle);
  }

  /** No per-loop logic required; the hardware controller handles movement. */
  @Override
  public void execute() {}

  /**
   * Stops the deflector when the command ends.
   *
   * @param interrupted Whether the command was interrupted before reaching the target
   */
  @Override
  public void end(boolean interrupted) {
    deflectorSubsystem.stop();
  }

  /**
   * Returns {@code true} once the deflector has stably reached its target angle.
   *
   * @return Whether the deflector is at the target angle
   */
  @Override
  public boolean isFinished() {
    return deflectorSubsystem.isAtTargetAngle();
  }
}
