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

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.DEFLECTOR_MAX_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Default command that continuously runs the deflector at a target angular velocity.
 *
 * <p>Accepts either a typed {@link Supplier} of {@link AngularVelocity} for programmatic use or a
 * raw {@link DoubleSupplier} representing a controller axis, which is scaled to the maximum
 * deflector velocity. This command never finishes on its own and is intended to be used as the
 * default command for the deflector subsystem.
 */
public class DefaultDeflectorRunAtVelocity extends Command {
  private final DeflectorSubsystem deflectorSubsystem;
  private final Supplier<AngularVelocity> targetVelocity;

  /**
   * Creates a new DefaultDeflectorRunAtVelocity command using a typed velocity supplier.
   *
   * @param deflectorSubsystem The deflector subsystem this command will control
   * @param targetVelocity Supplier that provides the desired angular velocity each loop
   */
  public DefaultDeflectorRunAtVelocity(
      DeflectorSubsystem deflectorSubsystem, Supplier<AngularVelocity> targetVelocity) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.targetVelocity = targetVelocity;

    addRequirements(deflectorSubsystem);
  }

  /**
   * Creates a new DefaultDeflectorRunAtVelocity command using a controller axis input.
   *
   * <p>The raw axis value is clamped to [{@code MIN_AXIS_LIMIT}, {@code MAX_AXIS_LIMIT}] and
   * scaled by {@code DEFLECTOR_MAX_VELOCITY} to produce the target angular velocity.
   *
   * @param deflectorSubsystem The deflector subsystem this command will control
   * @param controllerInput Supplier of a raw controller axis value in the range [-1.0, 1.0]
   */
  public DefaultDeflectorRunAtVelocity(
      DeflectorSubsystem deflectorSubsystem, DoubleSupplier controllerInput) {
    this.deflectorSubsystem = deflectorSubsystem;
    this.targetVelocity =
        () ->
            DEFLECTOR_MAX_VELOCITY.times(
                MathUtil.clamp(controllerInput.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT));

    addRequirements(deflectorSubsystem);
  }

  /** No initialization logic required for this command. */
  @Override
  public void initialize() {}

  /** Continuously commands the deflector to the velocity provided by the supplier. */
  @Override
  public void execute() {
    deflectorSubsystem.runToTargetVelocity(targetVelocity.get());
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
