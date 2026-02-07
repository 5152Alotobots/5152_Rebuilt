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
package frc.alotobots.rebuilt.subsystems.turret.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final double AT_TARGET_ANGLE_TIME_THRESHOLD = 0.1; // seconds
  public static final Angle AT_TARGET_ANGLE_THRESHOLD = Degrees.of(1); // degrees
  public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.05;
  public static final double MIN_OPEN_LOOP_PERCENTAGE = -0.05;
}
