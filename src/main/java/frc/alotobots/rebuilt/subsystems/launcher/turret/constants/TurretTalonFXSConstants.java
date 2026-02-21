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

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public class TurretTalonFXSConstants {
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 0.0;

  public static final double POSITION_P_GAIN = 15.0;
  public static final double POSITION_I_GAIN = 0.0;
  public static final double POSITION_D_GAIN = 4.0;

  // Gear ratio between the turret mechanism and the motor sensor 10 / 1 being a reduction
  public static final double SENSOR_TO_MECHANISM_RATIO = 14.4 / 1;
  public static final double ROBOT_TO_TURRET_OFFSET_X = -.05;
  public static final double ROBOT_TO_TURRET_OFFSET_Y = .15;
  public static final Angle MIN_ANGLE = Degrees.of(-90);
  public static final Angle MAX_ANGLE = Degrees.of(90);
  public static final double MOMENT_OF_INERTIA =
      .3; // TODO: Placeholder value, should be calculated based on the actual turret design
}
