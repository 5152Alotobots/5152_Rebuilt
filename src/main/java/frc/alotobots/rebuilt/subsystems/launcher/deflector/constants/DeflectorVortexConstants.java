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

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.Current;

public class DeflectorVortexConstants {
  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** Vortex-specific PID and motion control constants for Position mode (Position mode). */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double DEFLECTOR_POSITION_KP = 0.4;

      /** Position control integral gain */
      public static final double DEFLECTOR_POSITION_KI = 0.0;

      /** Position control derivative gain */
      public static final double DEFLECTOR_POSITION_KD = 0.0;

      /** Gravity compensation gain */
      public static final double DEFLECTOR_POSITION_KG = 0.0;

      /** The allowed closed-loop error in rotations */
      public static final double DEFLECTOR_POSITION_ALLOWED_CLOSED_LOOP_ERROR = 0.1;
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

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final SparkBaseConfig.IdleMode DEFLECTOR_MECHANISM_NEUTRAL_MODE =
      SparkBaseConfig.IdleMode.kBrake;

  public static final boolean DEFLECTOR_MOTOR_DIRECTION_INVERTED = false;
  public static final SensorDirectionValue DEFLECTOR_ENCODER_SENSOR_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;

  public static final double DEFLECTOR_ENCODER_MAGNET_OFFSET = 0.394287109375;
  public static final double DEFLECTOR_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.5;

  /**
   * Regression used to calculate how many radians the hood angle changes per motor rotation radian
   */
  public static final double DEFLECTOR_ROTATION_PER_ROTATION = -0.0749254;
}
