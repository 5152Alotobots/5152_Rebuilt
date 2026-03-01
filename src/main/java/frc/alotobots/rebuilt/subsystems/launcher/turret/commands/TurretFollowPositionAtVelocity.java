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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Command that continuously rotates the turret to a dynamically supplied angle.
 *
 * <p>Unlike {@link TurretRunToPosition}, this command does not finish when the target is reached.
 * Instead it re-evaluates the angle supplier every loop, making it suitable for continuously
 * tracking a moving target such as the hub, while applying a velocity feed-forward The command runs
 * indefinitely until interrupted.
 */
public class TurretFollowPositionAtVelocity extends Command {
  private final TurretSubsystem turretSubsystem;
  private final Supplier<Angle> angleSupplier;
  private final Supplier<AngularVelocity> angularVelocitySupplier;

  /**
   * Creates a new TurretFollowPosition command.
   *
   * @param turretSubsystem The turret subsystem this command will control
   * @param angleSupplier Supplier that provides the desired turret angle each loop
   */
  public TurretFollowPositionAtVelocity(
      TurretSubsystem turretSubsystem,
      Supplier<Angle> angleSupplier,
      Supplier<AngularVelocity> angularVelocitySupplier) {
    this.turretSubsystem = turretSubsystem;
    this.angleSupplier = angleSupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;

    addRequirements(turretSubsystem);
  }

  /** No initialization logic required for this command. */
  @Override
  public void initialize() {}

  /** Continuously commands the turret to the angle returned by the supplier. */
  @Override
  public void execute() {
    turretSubsystem.runToTargetAngleAtVelocity(angleSupplier.get(), angularVelocitySupplier.get());
  }

  /**
   * Stops the turret when the command ends.
   *
   * @param interrupted Whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
