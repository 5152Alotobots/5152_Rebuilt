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
import java.util.function.Supplier;

/**
 * Command that continuously moves the deflector to a dynamically supplied angle.
 *
 * <p>Unlike {@link DeflectorRunToPosition}, this command does not finish when the target is
 * reached. Instead it re-evaluates the angle supplier every loop, making it suitable for
 * continuously tracking a moving target such as an interpolated shot angle. The command runs
 * indefinitely until interrupted.
 */
public class DeflectorFollowPosition extends Command {
  private final DeflectorSubsystem deflectorSubsystem;
  private final Supplier<Angle> angleSupplier;

  /**
   * Creates a new DeflectorFollowPosition command.
   *
   * @param deflectorSubsystem The deflector subsystem this command will control
   * @param angleSupplier Supplier that provides the desired deflector angle each loop
   */
  public DeflectorFollowPosition(
      DeflectorSubsystem deflectorSubsystem, Supplier<Angle> angleSupplier) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.angleSupplier = angleSupplier;

    addRequirements(deflectorSubsystem);
  }

  /** No initialization logic required for this command. */
  @Override
  public void initialize() {}

  /** Continuously commands the deflector to the angle returned by the supplier. */
  @Override
  public void execute() {
    deflectorSubsystem.runToTargetAngle(angleSupplier.get());
  }

  /**
   * Stops the deflector when the command ends.
   *
   * @param interrupted Whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    deflectorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
