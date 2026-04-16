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
package frc.alotobots.rebuilt.subsystems.launcher.shooter.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

@UtilityClass
public class ShooterConstants {

  /** Contains threshold values for various shooter operations. */
  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at velocity" */
    public static final AngularVelocity SHOOTER_AT_TARGET_VELOCITY_SPEED_THRESHOLD =
        RotationsPerSecond.of(3.5);

    /** How long the flywheel must be "at velocity" to classify as "at velocity" */
    public static final Time SHOOTER_AT_TARGET_VELOCITY_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Contains physical limits and safety thresholds for the shooter. */
  public static final class Limits {
    /** Maximum open loop percent output */
    public static final double SHOOTER_MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final AngularVelocity SHOOTER_MAX_VELOCITY = RotationsPerSecond.of(90);

    /** Enable Limits */
    public static final boolean SHOOTER_LIMITS_ENABLED = true;
  }

  /** Contains velocity setpoints for different shooter states. */
  public static final class Setpoints {
    // Different setpoints would go here.
    public static final class Fixed {
      // TODO: real data
      public static final AngularVelocity FIXED_SHOOTER_VELOCITY_LEFT_TRENCH =
          RotationsPerSecond.of(50);
      public static final AngularVelocity FIXED_SHOOTER_VELOCITY_CENTER = RotationsPerSecond.of(50);
      public static final AngularVelocity FIXED_SHOOTER_VELOCITY_RIGHT_TRENCH =
          RotationsPerSecond.of(50);
    }
  }
}
