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
package frc.alotobots;

import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.alotobots.library.subsystems.swervedrive.constants.TunerConstants;
import frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023.TunerConstants2023;
import frc.alotobots.library.subsystems.swervedrive.constants.mk4i2025.TunerConstants2025;
import frc.alotobots.library.subsystems.swervedrive.constants.mk5i2026.TunerConstants2026;
import lombok.experimental.UtilityClass;

/**
 * Robot-wide constants class that defines runtime modes and device configurations. This class
 * contains global constants and configurations used across the robot code.
 */
public final class Constants {
  /** The simulation mode to use when not running on real hardware. */
  public static final Mode simMode = Mode.REPLAY;

  /**
   * The current runtime mode, determined by whether running on real hardware or in simulation.
   * Assume competition if real hardware and unspecified
   */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** The current robot hardware to deploy for */
  public static final Robot currentRobot = Robot.COMPETITION;

  /** Defines the physical robots for the robot code. */
  public enum Robot {
    /** COMPETITION swerve chassis */
    COMPETITION,
    /** DEV swerve chassis */
    DEV
  }

  /** Defines the possible runtime modes for the robot code. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final TunerConstants tunerConstants =
      switch (currentRobot) {
        case COMPETITION -> RobotBase.isReal() || currentMode == Mode.REPLAY
            ? new TunerConstants2026()
            : new TunerConstants2025();
        case DEV -> new TunerConstants2023();
      };

  public static final Time LOOP_PERIOD = Milliseconds.of(20);

  public static final Time LOOP_PERIOD_WATCHDOG = Milliseconds.of(200);

  /**
   * CAN bus device ID assignments. This class maps CAN IDs for all motors, sensors and other
   * CAN-connected devices.
   */
  @UtilityClass
  public static final class CanId {

    public static final CANBus SWERVE_CAN_BUS = new CANBus("Swerve");
    public static final CANBus RIO_CAN_BUS = new CANBus("rio");

    public static final double DEFAULT_CAN_FREQUENCY = 50.0;

    public static final int PDH_CAN_ID = 1;

    public static final int CANDLE_CAN_ID = 2;

    // CAN IDs 3, 10-21 reserved for swerve

    public static final int CLIMBER_CAN_ID = 30;
    public static final int INTAKE_EXTENDO_CAN_ID = 31;
    public static final int INTAKE_ROLLER_CAN_ID = 32;
    public static final int CONVEYOR_CAN_ID = 33;
    public static final int TURRET_CAN_ID = 34;
    public static final int DEFLECTOR_MOTOR_CAN_ID = 35;
    public static final int SHOOTER_LEFT_CAN_ID = 36;
    public static final int SHOOTER_RIGHT_CAN_ID = 37;
    public static final int KICKER_CAN_ID = 38;
    public static final int DEFLECTOR_ENCODER_CAN_ID = 40;
    public static final int ROLLER_MOTOR_CAN_ID = 41;
  }

  @UtilityClass
  public static final class DIO {
    public static final int TURRET_RESET_LIMIT_SWITCH_CHANNEL = 0;
  }
}
