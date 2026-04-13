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

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;

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

  public static final class PIDConstants {
    /** Vortex-specific PID and motion control constants for Position mode (Position mode). */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double DEFLECTOR_POSITION_KP = 0.8;

      /** Position control integral gain */
      public static final double DEFLECTOR_POSITION_KI = 0.0;

      /** Position control derivative gain */
      public static final double DEFLECTOR_POSITION_KD = 0.0;

      /** Gravity compensation gain */
      public static final double DEFLECTOR_POSITION_KG = 0.0;

      /** The allowed closed-loop error in rotations */
      public static final double DEFLECTOR_POSITION_ALLOWED_CLOSED_LOOP_ERROR = 0.02;
    }

    /** Vortex-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double DEFLECTOR_VELOCITY_KP = 0.0;

      /** Position control integral gain */
      public static final double DEFLECTOR_VELOCITY_KI = 0.0;

      /** Position control derivative gain */
      public static final double DEFLECTOR_VELOCITY_KD = 0.0;

      /** Gravity compensation gain */
      public static final double DEFLECTOR_VELOCITY_KG = 0.0;

      /** Static friction compensation */
      public static final double DEFLECTOR_VELOCITY_KS = 0.0;

      /** Velocity feedforward gain */
      public static final double DEFLECTOR_VELOCITY_KV = 0.0;

      /** The allowed closed-loop error in rotations */
      public static final double DEFLECTOR_VELOCITY_ALLOWED_CLOSED_LOOP_ERROR = 0.1;
    }
  }

  /** Contains safety limit constants for the motors. */
  public static final class MotorSafetyLimits {
    /** Maximum torque current limit in amperes */
    public static final Current DEFLECTOR_TORQUE_AMP_LIMIT = Amps.of(45);
  }

  public static final boolean DEFLECTOR_MOTOR_DIRECTION_INVERTED = false;

  public static final double DEFLECTOR_ENCODER_MAGNET_OFFSET = 0.453125;
  public static final double DEFLECTOR_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.5;

  /**
   * Regression used to calculate how many radians the hood angle changes per motor rotation radian
   */
  public static final double DEFLECTOR_ROTATION_PER_ROTATION = -0.0749254;

  /** Gear ratio between the deflector mechanism output and the motor sensor (100:1 reduction) */
  public static final double SENSOR_TO_MECHANISM_RATIO = 100.0 / 1.0;
}
