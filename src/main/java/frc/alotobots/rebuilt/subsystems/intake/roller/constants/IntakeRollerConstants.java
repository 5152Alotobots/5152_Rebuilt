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
package frc.alotobots.rebuilt.subsystems.intake.roller.constants;

import lombok.experimental.UtilityClass;

@UtilityClass
public class IntakeRollerConstants {
  /** Contains threshold values for various intake roller operations. */
  public static final class Thresholds {
    // Class is intentionally left empty
  }

  /** Contains physical limits and safety thresholds for the intake roller. */
  public static final class Limits {

    /** Maximum open loop percent output (global) */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 1;

    /** Maximum open loop intake percent output */
    public static final double MAX_OPEN_LOOP_INTAKE_PERCENTAGE = 1;

    /** Maximum open loop intake percent output */
    public static final double MAX_OPEN_LOOP_EJECT_PERCENTAGE = 0.75;
  }

  /** Setpoints for different roller states */
  public static final class Setpoints {
    public static final class OpenLoop {
      public static final double INTAKE_PERCENTAGE = 0.75;
      public static final double EJECT_PERCENTAGE = 0.75;
    }
  }
}
