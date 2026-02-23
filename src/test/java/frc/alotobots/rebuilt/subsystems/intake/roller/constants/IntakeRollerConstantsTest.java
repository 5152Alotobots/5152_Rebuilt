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

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class IntakeRollerConstantsTest {

  @Test
  void testLimitConstants() {
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE > 0);
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE <= 1.0);
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE > 0);
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE <= 1.0);
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE > 0);
    assertTrue(IntakeRollerConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE <= 1.0);
  }

  @Test
  void testLimitsEnabled() {
    assertNotNull(IntakeRollerConstants.Limits.LIMITS_ENABLED);
  }

  @Test
  void testSetpointConstants() {
    // Intake percentage should be within valid range
    assertTrue(IntakeRollerConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE >= 0);
    assertTrue(IntakeRollerConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE <= 1.0);

    // Eject percentage should be within valid range
    assertTrue(IntakeRollerConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE >= 0);
    assertTrue(IntakeRollerConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE <= 1.0);
  }

  @Test
  void testSetpointsWithinLimits() {
    // Intake setpoint should be within intake limit
    assertTrue(
        IntakeRollerConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE
            <= IntakeRollerConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE);

    // Eject setpoint should be within eject limit
    assertTrue(
        IntakeRollerConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE
            <= IntakeRollerConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE);
  }

  @Test
  void testIntakeLimitNotExceedingGlobalLimit() {
    assertTrue(
        IntakeRollerConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE
            <= IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE);
  }

  @Test
  void testEjectLimitNotExceedingGlobalLimit() {
    assertTrue(
        IntakeRollerConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE
            <= IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE);
  }
}