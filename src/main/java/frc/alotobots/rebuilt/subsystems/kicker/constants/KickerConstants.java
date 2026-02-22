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
package frc.alotobots.rebuilt.subsystems.kicker.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

@UtilityClass
public class KickerConstants {

  /** Contains threshold values for various wrist operations. */
  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at velocity" */
    public static final AngularVelocity AT_TARGET_VELOCITY_SPEED_THRESHOLD = DegreesPerSecond.of(3);

    /** How long the flywheel must be "at velocity" to classify as "at velocity" */
    public static final Time AT_TARGET_VELOCITY_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Contains physical limits and safety thresholds for the wrist. */
  public static final class Limits {
    /** Maximum open loop percent output */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final AngularVelocity MAX_SPEED = RotationsPerSecond.of(90);

    /** Enable Limits */
    public static final boolean LIMITS_ENABLED = true;
  }

  /** Contains position setpoints for different wrist states. */
  public static final class Setpoints {
    // Different setpoints would go here. Still coming up with a naming scheme. depends if we need
    // multiple speeds for more granular shooting control
    public static final AngularVelocity LOAD_INTO_SHOOTER_VELOCITY = RotationsPerSecond.of(90);
  }
}
