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

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;

public class TurretTalonFXSConstants {
  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {

    /** TalonFX-specific PID and motion control constants for Position mode (Position mode). */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double TURRET_POSITION_KP = 24.0;

      /** Position control integral gain */
      public static final double TURRET_POSITION_KI = 0.0;

      /** Position control derivative gain */
      public static final double TURRET_POSITION_KD = 0.0;

      /** Gravity compensation gain */
      public static final double TURRET_POSITION_KG = 0.0;
    }

    /** TalonFX-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double TURRET_VELOCITY_KP = 0.1;

      /** Position control integral gain */
      public static final double TURRET_VELOCITY_KI = 0.0;

      /** Position control derivative gain */
      public static final double TURRET_VELOCITY_KD = 0.0;

      /** Gravity compensation gain */
      public static final double TURRET_VELOCITY_KG = 0.0;

      /** Static friction compensation */
      public static final double TURRET_VELOCITY_KS = 0.0;

      /** Velocity feedforward gain */
      public static final double TURRET_VELOCITY_KV = 0.05;
    }
  }

  /** Contains safety limit constants for the motors. */
  public static final class MotorSafetyLimits {
    /** Maximum stator current limit in amperes */
    public static final Current TURRET_STATOR_AMP_LIMIT = Amps.of(45);
  }

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue TURRET_MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;

  public static final InvertedValue TURRET_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  // Gear ratio between the turret mechanism and the motor sensor 14.4 / 1 being a reduction
  public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = 14.4;
}
