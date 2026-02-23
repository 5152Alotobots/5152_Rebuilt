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

import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Hardware-specific constants for the deflector when driven by a TalonFXS motor controller.
 *
 * <p>Contains motor configuration values including neutral mode, encoder offset, PID gains, and the
 * sensor-to-mechanism gear ratio used by the TalonFXS onboard closed-loop controller.
 */
public class DeflectorTalonFXSConstants {
  /** Neutral mode applied to the TalonFXS when no output is commanded */
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

  /** Absolute encoder zero offset in rotations */
  public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 0.0;

  /** Position closed-loop proportional gain */
  public static final double POSITION_P_GAIN = .01;

  /** Position closed-loop integral gain */
  public static final double POSITION_I_GAIN = 0.0;

  /** Position closed-loop derivative gain */
  public static final double POSITION_D_GAIN = 0.0;

  /** Gear ratio between the deflector mechanism output and the motor sensor (100:1 reduction) */
  public static final double SENSOR_TO_MECHANISM_RATIO = 100.0 / 1.0;
}
