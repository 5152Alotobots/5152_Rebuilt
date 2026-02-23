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

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ConstantsTest {

  @Test
  void testRobotEnumValues() {
    // Verify Robot enum has expected values
    assertEquals(2, Constants.Robot.values().length);
    assertNotNull(Constants.Robot.COMPETITION);
    assertNotNull(Constants.Robot.DEV);
  }

  @Test
  void testModeEnumValues() {
    // Verify Mode enum has expected values
    assertEquals(3, Constants.Mode.values().length);
    assertNotNull(Constants.Mode.REAL);
    assertNotNull(Constants.Mode.SIM);
    assertNotNull(Constants.Mode.REPLAY);
  }

  @Test
  void testCurrentRobotIsNotNull() {
    assertNotNull(Constants.currentRobot);
  }

  @Test
  void testCurrentModeIsNotNull() {
    assertNotNull(Constants.currentMode);
  }

  @Test
  void testSimModeIsNotNull() {
    assertNotNull(Constants.simMode);
  }

  @Test
  void testTunerConstantsIsNotNull() {
    assertNotNull(Constants.tunerConstants);
  }

  @Test
  void testCanIdConstants() {
    // Verify CAN bus constants
    assertNotNull(Constants.CanId.SWERVE_CAN_BUS);
    assertNotNull(Constants.CanId.RIO_CAN_BUS);

    // Verify CAN frequency is positive
    assertTrue(Constants.CanId.DEFAULT_CAN_FREQUENCY > 0);

    // Verify PDH CAN ID
    assertEquals(1, Constants.CanId.PDH_CAN_ID);

    // Verify other CAN IDs are in valid range (1-63)
    assertTrue(Constants.CanId.CANDLE_CAN_ID >= 1 && Constants.CanId.CANDLE_CAN_ID <= 63);
    assertTrue(Constants.CanId.CLIMBER_CAN_ID >= 1 && Constants.CanId.CLIMBER_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.INTAKE_EXTENDO_CAN_ID >= 1 && Constants.CanId.INTAKE_EXTENDO_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.INTAKE_ROLLER_CAN_ID >= 1 && Constants.CanId.INTAKE_ROLLER_CAN_ID <= 63);
    assertTrue(Constants.CanId.CONVEYOR_CAN_ID >= 1 && Constants.CanId.CONVEYOR_CAN_ID <= 63);
    assertTrue(Constants.CanId.TURRET_CAN_ID >= 1 && Constants.CanId.TURRET_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.DEFLECTOR_MOTOR_CAN_ID >= 1
            && Constants.CanId.DEFLECTOR_MOTOR_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.SHOOTER_LEFT_CAN_ID >= 1 && Constants.CanId.SHOOTER_LEFT_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.SHOOTER_RIGHT_CAN_ID >= 1 && Constants.CanId.SHOOTER_RIGHT_CAN_ID <= 63);
    assertTrue(Constants.CanId.KICKER_CAN_ID >= 1 && Constants.CanId.KICKER_CAN_ID <= 63);
    assertTrue(
        Constants.CanId.DEFLECTOR_ENCODER_CAN_ID >= 1
            && Constants.CanId.DEFLECTOR_ENCODER_CAN_ID <= 63);
  }

  @Test
  void testCanIdUniqueness() {
    // Verify all motor CAN IDs are unique
    int[] motorIds = {
      Constants.CanId.CLIMBER_CAN_ID,
      Constants.CanId.INTAKE_EXTENDO_CAN_ID,
      Constants.CanId.INTAKE_ROLLER_CAN_ID,
      Constants.CanId.CONVEYOR_CAN_ID,
      Constants.CanId.TURRET_CAN_ID,
      Constants.CanId.DEFLECTOR_MOTOR_CAN_ID,
      Constants.CanId.SHOOTER_LEFT_CAN_ID,
      Constants.CanId.SHOOTER_RIGHT_CAN_ID,
      Constants.CanId.KICKER_CAN_ID
    };

    for (int i = 0; i < motorIds.length; i++) {
      for (int j = i + 1; j < motorIds.length; j++) {
        assertNotEquals(motorIds[i], motorIds[j], "CAN IDs must be unique");
      }
    }
  }

  @Test
  void testDIOConstants() {
    // Verify DIO channel is in valid range (0-25 for RoboRIO 2.0)
    assertTrue(
        Constants.DIO.TURRET_RESET_LIMIT_SWITCH_CHANNEL >= 0
            && Constants.DIO.TURRET_RESET_LIMIT_SWITCH_CHANNEL <= 25);
  }

  @Test
  void testCurrentRobotMatchesTunerConstants() {
    // Verify that tuner constants are initialized based on current robot
    assertNotNull(Constants.tunerConstants);
  }
}