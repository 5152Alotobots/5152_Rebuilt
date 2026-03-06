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
package frc.alotobots.rebuilt.subsystems.intake.roller.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

/**
 * Constants for the physical intake roller subsystem using TalonFX motors. Contains PID constants
 * for different control modes and motor safety limits.
 */
@UtilityClass
public class IntakeRollerTalonFXConstants {

  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    // This class is intentionally left empty
  }

  /** Contains safety limit constants for the intake roller motor. */
  public static final class MotorSafetyLimits {
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(20);
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(-20);
    public static final Current STATOR_AMP_LIMIT = Amps.of(20);
    public static final Current SUPPLY_PEAK_LIMIT = Amps.of(25);
    public static final Current SUPPLY_SUSTAINED_LIMIT = Amps.of(20);
    public static final Time SUPPLY_PEAK_DURATION = Seconds.of(0.25);
  }

  /** Direction of the intake roller motor rotation */
  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;
}
