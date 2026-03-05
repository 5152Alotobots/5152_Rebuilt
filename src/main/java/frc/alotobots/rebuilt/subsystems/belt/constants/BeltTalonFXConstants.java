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
package frc.alotobots.rebuilt.subsystems.belt.constants;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class BeltTalonFXConstants {
  public static final double VELOCITY_P_GAIN = 2.5;
  public static final double VELOCITY_I_GAIN = 0.0;
  public static final double VELOCITY_D_GAIN = 0.0;
  public static final double VELOCITY_V_GAIN = 0.125;
  public static final double VELOCITY_S_GAIN = 0.0090433;
  public static final double CLOSED_LOOP_RAMP_RATE = 0.5;

  /** Contains safety limit constants for the motors. */
  public static final class MotorSafetyLimits {
    public static final Current BELT_TORQUE_FORWARD_AMP_LIMIT = Amps.of(30);
    public static final Current BELT_TORQUE_REVERSE_AMP_LIMIT = Amps.of(-30);
    public static final Current BELT_STATOR_AMP_LIMIT = Amps.of(30);
  }
}
