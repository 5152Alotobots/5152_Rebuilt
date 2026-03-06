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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

@UtilityClass
public class DeflectorConstants {

  /** Contains threshold values for various deflector operations. */
  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at position" */
    public static final Angle DEFLECTOR_AT_TARGET_ANGLE_POSITION_THRESHOLD = Degrees.of(3);

    /** How long the wrist must be "at position" to classify as "at position" */
    public static final Time DEFLECTOR_AT_TARGET_ANGLE_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Contains physical limits and safety thresholds for the wrist. */
  public static final class Limits {
    /** Maximum allowed angle */
    public static final Angle DEFLECTOR_MAX_ANGLE = Degrees.of(68);

    /** Minimum allowed angle */
    public static final Angle DEFLECTOR_MIN_ANGLE = Degrees.of(41.09268);

    /** Maximum open loop percent output */
    public static final double DEFLECTOR_MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final AngularVelocity DEFLECTOR_MAX_VELOCITY = DegreesPerSecond.of(90);

    /** Enable Limits */
    public static final boolean DEFLECTOR_LIMITS_ENABLED = true;
  }

  /** Contains position setpoints for different wrist states. */
  public static final class Setpoints {
    // Different setpoints would go here. Still coming up with a naming scheme
    public static final class Fixed {
      // TODO: tune
      public static final Angle FIXED_DEFLECTOR_ANGLE_LEFT_TRENCH = Degrees.of(50);
      public static final Angle FIXED_DEFLECTOR_ANGLE_CENTER = Degrees.of(50);
      public static final Angle FIXED_DEFLECTOR_ANGLE_RIGHT_TRENCH = Degrees.of(50);
    }
  }
}
