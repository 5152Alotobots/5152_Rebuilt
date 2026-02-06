/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.deflector.constnats;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeflectorTalonFXSConstants {
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 0.0;

  public static final double POSITION_P_GAIN = .01;
  public static final double POSITION_I_GAIN = 0.0;
  public static final double POSITION_D_GAIN = 0.0;

  // Gear ratio between the turret mechanism and the motor sensor 10 / 1 being a reduction
  public static final double SENSOR_TO_MECHANISM_RATIO = 100.0 / 1.0;
}
