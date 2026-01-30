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
package frc.alotobots.rebuilt.subsystems.launcher.io;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOTalonFX implements ShooterIO {
  public ShooterIOTalonFX() {
    // Constructor implementation
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    // Update inputs implementation
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
    // Set shooter velocity with PID slot implementation
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    // Set shooter velocity implementation
    setShooterVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
  }

  @Override
  public void setShooterOpenLoop(double percentOutput) {
    // Set shooter open loop implementation
  }

  @Override
  public void stop() {
    // Stop shooter implementation
  }
}
