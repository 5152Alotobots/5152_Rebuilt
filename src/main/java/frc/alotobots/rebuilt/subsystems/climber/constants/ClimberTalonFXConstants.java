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
package frc.alotobots.rebuilt.subsystems.climber.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimberTalonFXConstants {

  public static final class PIDConstants {
    public static final class VelocityPIDConstants {
      // TODO tune
      public static final double KP = 0.1;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KG = 0.1;
      public static final double KS = 0.1;
      public static final double KV = 0.1;
    }

    public static final class PositionPIDConstants {
      // TODO tune
      public static final double KP = 10;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KF = 0.0;
      public static final double KG = 0;
      public static final double KS = 0;
      public static final double KV = 0;
    }

    public static final class MotionMagicPositionPIDConstants {
      // TODO tune
      public static final double KP = 0.1;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KF = 0.05;
    }
  }

  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final class MotionMagicConstants {
    public static final LinearVelocity CRUISE_VELOCITY = MetersPerSecond.of(0);
    public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(0);
    public static final double JERK = 0.0;
  }

  public static final class MotorSafetyLimits {
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(40.0);
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(40.0);
    public static final Current STATOR_AMP_LIMIT = Amps.of(800.0);
  }
}
