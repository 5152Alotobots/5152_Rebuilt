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

import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Hardware-specific constants for the turret when driven by a SPARK MAX motor controller.
 *
 * <p>Contains motor configuration values including neutral (idle) mode and the absolute encoder
 * zero offset used to home the turret on startup.
 */
public class TurretSparkMaxConstants {
  /** Idle mode applied to the SPARK MAX when no output is commanded */
  public static final SparkBaseConfig.IdleMode NEUTRAL_MODE = SparkBaseConfig.IdleMode.kBrake;

  /** Absolute encoder zero offset in rotations */
  public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 0.0;
}
