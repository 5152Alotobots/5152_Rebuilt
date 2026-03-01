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
package frc.alotobots.rebuilt.subsystems.climber.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ClimberConstants {
  public static final double EXTENSION_PER_ROTATION = 0.00731707;

  public static class Limits {
    public static final Distance MIN_CLIMB_EXTENSION = Meters.of(0.48);
    public static final Distance MAX_CLIMB_EXTENSION = Meters.of(0.69); // Example value

    public static final Distance DISTANCE_FROM_LIMIT = Centimeters.of(2.0);
    public static final LinearVelocity MAX_VELOCITY_NEAR_LIMIT = MetersPerSecond.of(0.1);
  }

  public static class Thresholds {
    public static final Distance AT_TARGET_CLIMB_POSITION_THRESHOLD = Centimeters.of(1.0);
    public static final Time AT_TARGET_CLIMB_TIME_THRESHOLD = Seconds.of(0.1);
  }
}
