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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class BeltConstantsTest {

  @Test
  void testThresholdConstants() {
    // Verify threshold constants are positive
    assertTrue(
        BeltConstants.Thresholds.AT_TARGET_VELOCITY_SPEED_THRESHOLD.in(DegreesPerSecond) > 0);
    assertTrue(BeltConstants.Thresholds.AT_TARGET_VELOCITY_TIME_THRESHOLD.in(Seconds) > 0);
  }

  @Test
  void testLimitConstants() {
    // Verify limits are positive and reasonable
    assertTrue(BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE > 0);
    assertTrue(BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE <= 1.0);
    assertTrue(BeltConstants.Limits.MAX_SPEED.in(RotationsPerSecond) > 0);
  }

  @Test
  void testSetpointConstants() {
    // Verify setpoint velocities are positive
    assertTrue(BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY.in(RotationsPerSecond) > 0);
  }

  @Test
  void testLimitsEnabled() {
    // Verify limits enabled is a boolean
    assertNotNull(BeltConstants.Limits.LIMITS_ENABLED);
  }

  @Test
  void testSetpointWithinLimits() {
    // Verify setpoint is within MAX_SPEED
    assertTrue(
        BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY.in(RotationsPerSecond)
            <= BeltConstants.Limits.MAX_SPEED.in(RotationsPerSecond));
  }

  @Test
  void testMaxOpenLoopPercentageReasonable() {
    // Max open loop percentage should be between 0 and 1
    assertTrue(BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE >= 0.0);
    assertTrue(BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE <= 1.0);
  }
}