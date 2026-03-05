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
package frc.alotobots.rebuilt.util.hubshift;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;

public final class HubShiftUtilConstants {
  public static final Time MIN_FUEL_COUNT_DELAY = Seconds.of(1.0);
  public static final Time MAX_FUEL_COUNT_DELAY = Seconds.of(2.0);
  public static final Time SHIFT_END_FUEL_COUNT_EXTENSION = Seconds.of(3.0);
  public static final Time MIN_FUEL_TIME_OF_FLIGHT = LaunchCalculator.getMinHubFuelTimeOfFlight();
  public static final Time MAX_FUEL_TIME_OF_FLIGHT = LaunchCalculator.getMaxHubFuelTimeOfFlight();
  public static final Time APPROACHING_ACTIVE_FUDGE =
      MIN_FUEL_TIME_OF_FLIGHT.plus(MIN_FUEL_COUNT_DELAY).times(-1);
  public static final Time ENDING_ACTIVE_FUDGE =
      SHIFT_END_FUEL_COUNT_EXTENSION.plus(
          MAX_FUEL_TIME_OF_FLIGHT.plus(MAX_FUEL_COUNT_DELAY).times(-1));

  public static final Time AUTO_END_TIME = Seconds.of(20.0);
  public static final Time TELEOP_DURATION = Seconds.of(140.0);
  public static final boolean[] ACTIVE_SCHEDULE = {true, true, false, true, false, true};
  public static final boolean[] INACTIVE_SCHEDULE = {true, false, true, false, true, true};

  public static final Time APPROACHING_ACTIVE_NOTIFICAITON_TIME = Seconds.of(5);
  public static final Time APPROACHING_INACTIVE_NOTIFICATION_TIME = Seconds.of(5);

  public static final Time[] SHIFT_START_TIMES = {
    Seconds.of(0.0),
    Seconds.of(10.0),
    Seconds.of(35.0),
    Seconds.of(60.0),
    Seconds.of(85.0),
    Seconds.of(110.0)
  };
  public static final Time[] SHIFT_END_TIMES = {
    Seconds.of(10.0),
    Seconds.of(35.0),
    Seconds.of(60.0),
    Seconds.of(85.0),
    Seconds.of(110.0),
    Seconds.of(140.0)
  };

  public static final Time[] STARTING_ACTIVE_START_TIMES =
      new Time[] {
        Seconds.of(0.0),
        Seconds.of(10.0),
        Seconds.of(35.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(60.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(85.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(110.0).plus(APPROACHING_ACTIVE_FUDGE)
      };
  public static final Time[] STARTING_ACTIVE_END_TIMES =
      new Time[] {
        Seconds.of(10.0),
        Seconds.of(35.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(60.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(85.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(110.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(140.0)
      };
  public static final Time[] STARTING_INACTIVE_START_TIMES =
      new Time[] {
        Seconds.of(0.0),
        Seconds.of(10.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(35.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(60.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(85.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(110.0)
      };
  public static final Time[] STARTING_INACTIVE_END_TIMES =
      new Time[] {
        Seconds.of(10.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(35.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(60.0).plus(ENDING_ACTIVE_FUDGE),
        Seconds.of(85.0).plus(APPROACHING_ACTIVE_FUDGE),
        Seconds.of(110.0),
        Seconds.of(140.0)
      };
}
