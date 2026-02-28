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
package frc.alotobots.rebuilt.subsystems.launcher.shooter.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import lombok.experimental.UtilityClass;

@UtilityClass
public class ShooterTalonFXConstants {
  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** TalonFX-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double SHOOTER_VELOCITY_KP = 2.5;

      /** Position control integral gain */
      public static final double SHOOTER_VELOCITY_KI = 0.0;

      /** Position control derivative gain */
      public static final double SHOOTER_VELOCITY_KD = 0.0;

      /** Gravity compensation gain */
      public static final double SHOOTER_VELOCITY_KG = 0.0;

      /** Static friction compensation */
      public static final double SHOOTER_VELOCITY_KS = 0.0090433;

      /** Velocity feedforward gain */
      public static final double SHOOTER_VELOCITY_KV = 0.125;
    }
  }

  /** Contains safety limit constants for the motors. */
  public static final class MotorSafetyLimits {
    /** Maximum forward torque current limit in amperes */
    public static final Current SHOOTER_TORQUE_FORWARD_AMP_LIMIT = Amps.of(45);

    /** Maximum reverse torque current limit in amperes */
    public static final Current SHOOTER_TORQUE_REVERSE_AMP_LIMIT = Amps.of(-45);

    /** Maximum stator current limit in amperes */
    public static final Current SHOOTER_STATOR_AMP_LIMIT = Amps.of(45);
  }

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue SHOOTER_MECHANISM_NEUTRAL_MODE = NeutralModeValue.Coast;

  public static final InvertedValue SHOOTER_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
}
