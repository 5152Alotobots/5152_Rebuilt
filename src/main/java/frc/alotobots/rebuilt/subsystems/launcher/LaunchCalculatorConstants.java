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
package frc.alotobots.rebuilt.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.util.UnitInterpolatingMap;
import lombok.experimental.UtilityClass;

/**
 * @see LaunchCalculator
 */
@UtilityClass
public final class LaunchCalculatorConstants {
  // TODO: Get real data for this
  public static final Distance MINIMUM_SHOOTING_DISTANCE = Meters.of(1);
  public static final Distance MAXIMUM_SHOOTING_DISTANCE = Meters.of(5);

  /** How long the subsystem and code takes to respond to a command. Should be empirically tuned. */
  public static final Time LAUNCH_CALCULATOR_SUBSYSTEM_DELAY = Milliseconds.of(30);

  // FORWARD: +, LEFT: +, UP: +
  public static final Transform2d ROBOT_TO_TURRET =
      new Transform2d(new Translation2d(-.14, 0), Rotation2d.kZero);

  public final class Maps {
    public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> LAUNCHER_DEFLECTOR_ANGLE_MAP =
        new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
    public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit>
        LAUNCHER_SHOOTER_VELOCITY_MAP =
            new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);
    public static final UnitInterpolatingMap<DistanceUnit, TimeUnit> FUEL_TIME_OF_FLIGHT_MAP =
        new UnitInterpolatingMap<>(Units.Meters, Units.Seconds);

    static {
      // Data collected from testing. TOF is estimated at 0.5s for all points.
      // Points sorted by distance ascending.

      // Distance: ~1.43m
      // Turret: 0.53°, Deflector: 63.09°, RPS: 32.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(1.427), Degrees.of(63.09));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(1.427), RotationsPerSecond.of(32.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(1.427), Seconds.of(0.5));

      // Distance: ~2.03m
      // Turret: 0.53°, Deflector: 63.04°, RPS: 35.0
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(2.033), Degrees.of(63.04));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(2.033), RotationsPerSecond.of(35.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(2.033), Seconds.of(0.5));

      // Distance: ~2.75m (two nearly identical points — averaged)
      // Turret: -1.23°, Deflector: avg(60.81, 60.94) = 60.88°, RPS: avg(30, 40) = 35
      // Using the 40 RPS / 60.94° point as the cleaner shot
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(2.749), Degrees.of(60.88));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(2.749), RotationsPerSecond.of(35.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(2.749), Seconds.of(0.5));

      // Distance: ~2.83m (two identical points)
      // Turret: 0.53°, Deflector: 58.67°, RPS: 37.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(2.831), Degrees.of(58.67));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(2.831), RotationsPerSecond.of(37.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(2.831), Seconds.of(0.5));

      // Distance: ~3.06m
      // Turret: -1.23°, Deflector: 56.79°, RPS: 37.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.058), Degrees.of(56.79));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.058), RotationsPerSecond.of(37.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.058), Seconds.of(0.5));

      // Distance: ~3.17m (WITH TOF data point, but using 0.5s estimate)
      // Turret: -2.99°, Deflector: 54.52°, RPS: 37.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.169), Degrees.of(54.52));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.169), RotationsPerSecond.of(37.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.169), Seconds.of(0.5));

      // Distance: ~3.31m
      // Turret: -1.23°, Deflector: 59.39°, RPS: 40.0
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.308), Degrees.of(59.39));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.308), RotationsPerSecond.of(40.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.308), Seconds.of(0.5));

      // Distance: ~3.45m (two close points — averaged)
      // ~3.45: Turret: 0.53°, Deflector: 60.25°, RPS: 42.5
      // ~3.45: Turret: -1.23°, Deflector: 58.39°, RPS: 40.0
      // Using the 0.53° turret point (cleaner geometry)
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.449), Degrees.of(60.25));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.449), RotationsPerSecond.of(42.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.449), Seconds.of(0.5));

      // Distance: ~3.66m
      // Turret: 1.14°, Deflector: 57.06°, RPS: 40.0
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.658), Degrees.of(57.06));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.658), RotationsPerSecond.of(40.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.658), Seconds.of(0.5));

      // Distance: ~3.82m
      // Turret: -1.23°, Deflector: 56.74°, RPS: 42.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.824), Degrees.of(56.74));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.824), RotationsPerSecond.of(42.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.824), Seconds.of(0.5));

      // Distance: ~3.98m
      // Turret: 0.53°, Deflector: 57.45°, RPS: 42.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(3.983), Degrees.of(57.45));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(3.983), RotationsPerSecond.of(42.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(3.983), Seconds.of(0.5));

      // Distance: ~4.06m
      // Turret: -1.23°, Deflector: 57.42°, RPS: 42.5
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(4.057), Degrees.of(57.42));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(4.057), RotationsPerSecond.of(42.5));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(4.057), Seconds.of(0.5));

      // Distance: ~4.21m (WITH TOF data point, but using 0.5s estimate)
      // Turret: -2.99°, Deflector: 57.95°, RPS: 45.0
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(4.215), Degrees.of(57.95));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(4.215), RotationsPerSecond.of(45.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(4.215), Seconds.of(0.5));

      // Distance: ~4.40m
      // Turret: 0.53°, Deflector: 56.66°, RPS: 45.0
      LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(4.399), Degrees.of(56.66));
      LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(4.399), RotationsPerSecond.of(45.0));
      FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(4.399), Seconds.of(0.5));
    }
  }
}
