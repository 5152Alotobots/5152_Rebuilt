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
package frc.alotobots.rebuilt.subsystems.kicker.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class KickerConstantsTest {

  @Test
  void testThresholdConstants() {
    assertTrue(
        KickerConstants.Thresholds.AT_TARGET_VELOCITY_SPEED_THRESHOLD.in(DegreesPerSecond) > 0);
    assertTrue(KickerConstants.Thresholds.AT_TARGET_VELOCITY_TIME_THRESHOLD.in(Seconds) > 0);
  }

  @Test
  void testLimitConstants() {
    assertTrue(KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE > 0);
    assertTrue(KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE <= 1.0);
    assertTrue(KickerConstants.Limits.MAX_SPEED.in(RotationsPerSecond) > 0);
  }

  @Test
  void testSetpointConstants() {
    assertTrue(KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY.in(RotationsPerSecond) > 0);
  }

  @Test
  void testLimitsEnabled() {
    assertNotNull(KickerConstants.Limits.LIMITS_ENABLED);
  }

  @Test
  void testSetpointWithinLimits() {
    assertTrue(
        KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY.in(RotationsPerSecond)
            <= KickerConstants.Limits.MAX_SPEED.in(RotationsPerSecond));
  }

  @Test
  void testMaxOpenLoopPercentageReasonable() {
    assertTrue(KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE >= 0.0);
    assertTrue(KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE <= 1.0);
  }

  @Test
  void testTimeThresholdReasonable() {
    // Time threshold should be less than 1 second for responsiveness
    assertTrue(KickerConstants.Thresholds.AT_TARGET_VELOCITY_TIME_THRESHOLD.in(Seconds) < 1.0);
  }
}