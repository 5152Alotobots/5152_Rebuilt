/*
 * ALOTOBOTS - FRC Team 5152
 * https://github.com/5152Alotobots
 * Copyright (C) 2026 ALOTOBOTS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Source code must be publicly available on GitHub or an alternative web accessible site
 */
package frc.alotobots.rebuilt.subsystems.roller.constants;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;

public final class RollerSparkMaxConstants {
  
  // ==============================================================================
  // Motor Configuration
  // ==============================================================================
  public static final IdleMode ROLLER_MECHANISM_NEUTRAL_MODE = IdleMode.kBrake;
  public static final boolean ROLLER_MOTOR_DIRECTION_INVERTED = false;

  // ==============================================================================
  // Safety Limits
  // ==============================================================================
  public static final class MotorSafetyLimits {
    // 40 Amps is a safe default for a NEO/NEO Vortex, but tune based on your breaker
    public static final Current ROLLER_TORQUE_AMP_LIMIT = Amps.of(40);
  }

  // ==============================================================================
  // Closed-Loop Control (PIDF)
  // ==============================================================================
  public static final class PIDConstants {
    
    public static final class VelocityPIDConstants {
      // TODO: Tune these for your specific roller mass and gearing
      public static final double ROLLER_VELOCITY_KP = 0.0;
      public static final double ROLLER_VELOCITY_KI = 0.0;
      public static final double ROLLER_VELOCITY_KD = 0.0;
      
      // Feedforward terms
      public static final double ROLLER_VELOCITY_KS = 0.0; 
      public static final double ROLLER_VELOCITY_KV = 0.0; 
      public static final double ROLLER_VELOCITY_KG = 0.0; // Usually 0 for a roller unless it fights gravity

      // Allowed error in Rotations per Second (or RPM if you scaled the encoder)
      public static final double ROLLER_VELOCITY_ALLOWED_CLOSED_LOOP_ERROR = 0.0;
    }
  }
}
