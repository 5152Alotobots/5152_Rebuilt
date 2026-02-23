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
package frc.alotobots.rebuilt.subsystems.launcher.turret.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import lombok.experimental.UtilityClass;

@UtilityClass
public class TurretConstants {
  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at position" */
    public static final Angle AT_TARGET_ANGLE_POSITION_THRESHOLD = Degrees.of(2);

    /** How long the turret must be "at position" to classify as "at position" */
    public static final Time AT_TARGET_ANGLE_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Contains physical limits and safety thresholds for the turret. */
  public static final class Limits {
    /** Maximum allowed angle */
    public static final Angle MAX_ANGLE = Degrees.of(360);

    /** Minimum allowed angle */
    public static final Angle MIN_ANGLE = Degrees.of(0);

    /** Maximum open loop percent output */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(180);

    /** Enable Limits */
    public static final boolean LIMITS_ENABLED = true;
  }

  /** Contains position setpoints for different turret states. */
  public static final class Setpoints {
    // setpoints would go here, but I haven't figured them out yet
  }
}
