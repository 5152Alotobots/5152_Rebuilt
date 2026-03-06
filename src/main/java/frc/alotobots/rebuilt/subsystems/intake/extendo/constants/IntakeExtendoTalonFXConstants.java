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
package frc.alotobots.rebuilt.subsystems.intake.extendo.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class IntakeExtendoTalonFXConstants {
  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** TalonFX-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 0.1;

      /** Position control integral gain */
      public static final double KI = 0.0;

      /** Position control derivative gain */
      public static final double KD = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.0;

      /** Static friction compensation */
      public static final double KS = 0.0;

      /** Velocity feedforward gain */
      public static final double KV = 0.05;
    }

    /** TalonFX-specific PID and motion control constants for Position mode (Position mode). */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 2.3;

      /** Position control integral gain */
      public static final double KI = 0.0;

      /** Position control derivative gain */
      public static final double KD = 0.4;

      /** Acceleration feedforward gain */
      public static final double KA = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.31;

      /** Static friction compensation */
      public static final double KS = 0.19;

      /** Velocity feedforward gain */
      public static final double KV = 0.0;
    }
  }

  public static final class MotionMagicConstants {
    public static final LinearVelocity CRUISE_VELOCITY = MetersPerSecond.of(2.8);
    public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(3.6);
    public static final double JERK = 0;
  }

  /** Contains safety limit constants for the motors. */
  public static final class MotorSafetyLimits {
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(45);
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(-45);
    public static final Current STATOR_AMP_LIMIT = Amps.of(65);
    public static final Current SUPPLY_PEAK_LIMIT = Amps.of(55);
    public static final Current SUPPLY_SUSTAINED_LIMIT = Amps.of(45);
    public static final Time SUPPLY_PEAK_DURATION = Seconds.of(0.75);
  }

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  /**
   * Regression used to calculate extension of motor. (Should be linear) Rotations:Meters This
   * assumes we are measuring the opposite side of the triangle created by the intake (the one
   * parallel to the floor or that measures distance extended from the frame)
   */
  public static final double EXTENSION_PER_ROTATION = 0.0140327;
}
