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

import edu.wpi.first.units.measure.Angle;

public class DeflectorIOTalonFX implements DeflectorIO {
  public DeflectorIOTalonFX() {
    // Constructor implementation
  }

  @Override
  public void updateInputs(DeflectorIO.DeflectorIOInputs inputs) {
    // Update inputs implementation
  }

  @Override
  public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
    // Set deflector position with PID slot implementation
  }

  @Override
  public void setDeflectorOpenLoop(double percentOutput) {
    // Set deflector open loop implementation
  }

  @Override
  public void stop() {
    // Stop deflector implementation
  }
}
