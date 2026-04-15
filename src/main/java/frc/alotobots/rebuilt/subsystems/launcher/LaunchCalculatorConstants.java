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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.util.UnitInterpolatingMap;
import java.util.LinkedList;
import java.util.List;
import lombok.experimental.UtilityClass;

/**
 * @see LaunchCalculator
 */
@UtilityClass
public final class LaunchCalculatorConstants {
  // TODO: Get real data for this
  public static final Distance MINIMUM_HUB_SHOOTING_DISTANCE = Meters.of(1);
  public static final Distance MAXIMUM_HUB_SHOOTING_DISTANCE = Meters.of(6);

  public static final Distance MINIMUM_PASSING_SHOOTING_DISTANCE = Meters.of(3.5);
  public static final Distance MAXIMUM_PASSING_SHOOTING_DISTANCE = Meters.of(13);

  /** How long the subsystem and code takes to respond to a command. Should be empirically tuned. */
  public static final Time LAUNCH_CALCULATOR_SUBSYSTEM_DELAY = Milliseconds.of(30);

  // FORWARD: +, LEFT: +, UP: +
  public static final Transform2d ROBOT_TO_TURRET =
      new Transform2d(new Translation2d(-.14, 0), Rotation2d.kZero);

  public static final Translation3d PASSING_TARGET_DEPOT_BLUE = new Translation3d(1.5, 6.05, 0);
  public static final Translation3d PASSING_TARGET_OUTPOST_BLUE = new Translation3d(1.5, 1.95, 0);

  public static final Translation3d PASSING_TARGET_DEPOT_RED = new Translation3d(14.889, 6.05, 0);
  public static final Translation3d PASSING_TARGET_OUTPOST_RED = new Translation3d(14.889, 1.95, 0);

  public static final List<Translation2d> PASSING_TARGET_OPTIONS_BLUE =
      new LinkedList<>() {
        {
          add(PASSING_TARGET_DEPOT_BLUE.toTranslation2d());
          add(PASSING_TARGET_OUTPOST_BLUE.toTranslation2d());
        }
      };

  public static final List<Translation2d> PASSING_TARGET_OPTIONS_RED =
      new LinkedList<>() {
        {
          add(PASSING_TARGET_DEPOT_RED.toTranslation2d());
          add(PASSING_TARGET_OUTPOST_RED.toTranslation2d());
        }
      };

  public final class Maps {
    public static final UnitInterpolatingMap<DistanceUnit, AngleUnit>
        LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP = new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
    public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit>
        LAUNCHER_SHOOTER_HUB_VELOCITY_MAP =
            new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);
    public static final UnitInterpolatingMap<DistanceUnit, TimeUnit> FUEL_HUB_TIME_OF_FLIGHT_MAP =
        new UnitInterpolatingMap<>(Units.Meters, Units.Seconds);

    // TODO: update this
    static {
      // Points sorted by distance ascending. HUB UPDATED 3/22/26

      // LEGACY POINTS (NOT ENOUGH NEW DATA)
      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(1.427), Degrees.of(60.09));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(1.427), RotationsPerSecond.of(35));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(2.749), Degrees.of(57.88));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(2.749), RotationsPerSecond.of(37.5));

      // NEW POINTS
      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(2.202626), Degrees.of(62.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(2.202626), RotationsPerSecond.of(35));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(2.202626), Seconds.of(1.034));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(3.013908), Degrees.of(58.0));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(3.013908), RotationsPerSecond.of(37.5));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(3.013908), Seconds.of(1.06));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(3.185211), Degrees.of(57.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(3.185211), RotationsPerSecond.of(37.5));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(3.185211), Seconds.of(1.067));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(3.527779), Degrees.of(57.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(3.527779), RotationsPerSecond.of(40));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(3.527779), Seconds.of(1.08));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(3.949270), Degrees.of(57.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(3.949270), RotationsPerSecond.of(40));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(3.949270), Seconds.of(1.1));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(4.215489), Degrees.of(57.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(4.215489), RotationsPerSecond.of(42.5));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(4.215489), Seconds.of(1.167));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(4.426129), Degrees.of(58.5));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(4.426129), RotationsPerSecond.of(45.0));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(4.426129), Seconds.of(1.2166667));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(5.070521), Degrees.of(57.500000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(5.070521), RotationsPerSecond.of(47.5));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(5.070521), Seconds.of(1.15));

      LAUNCHER_DEFLECTOR_HUB_ANGLE_MAP.put(Meters.of(5.462477), Degrees.of(55.000000));
      LAUNCHER_SHOOTER_HUB_VELOCITY_MAP.put(Meters.of(5.462477), RotationsPerSecond.of(47.5));
      FUEL_HUB_TIME_OF_FLIGHT_MAP.put(Meters.of(5.462477), Seconds.of(1.2166667));
    }

    public static final UnitInterpolatingMap<DistanceUnit, AngleUnit>
        LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP =
            new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
    public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit>
        LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP =
            new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);
    public static final UnitInterpolatingMap<DistanceUnit, TimeUnit>
        FUEL_PASSING_TIME_OF_FLIGHT_MAP = new UnitInterpolatingMap<>(Units.Meters, Units.Seconds);

    static {
      // Points sorted by distance ascending. PASSING UPDATED 3/22/26
      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(4.985827), Degrees.of(55.0));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(4.985827), RotationsPerSecond.of(37.5));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(4.985827), Seconds.of(1.31));

      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(5.282597), Degrees.of(53.0));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(5.282597), RotationsPerSecond.of(40.0));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(5.282597), Seconds.of(1.32));

      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(6.506284), Degrees.of(50.5));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(6.506284), RotationsPerSecond.of(45.0));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(6.506284), Seconds.of(1.53));

      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(7.383009), Degrees.of(53.0));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(7.383009), RotationsPerSecond.of(47.5));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(7.383009), Seconds.of(1.6));

      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(7.971714), Degrees.of(50.5));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(7.971714), RotationsPerSecond.of(50.0));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(7.971714), Seconds.of(1.6));

      LAUNCHER_DEFLECTOR_PASSING_ANGLE_MAP.put(Meters.of(9.214514), Degrees.of(48.0));
      LAUNCHER_SHOOTER_PASSING_VELOCITY_MAP.put(Meters.of(9.214514), RotationsPerSecond.of(57.5));
      FUEL_PASSING_TIME_OF_FLIGHT_MAP.put(Meters.of(9.214514), Seconds.of(1.74));
    }
  }
}
