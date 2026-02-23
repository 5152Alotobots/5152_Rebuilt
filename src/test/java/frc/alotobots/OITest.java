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

class OITest {

  @Test
  void testDeadbandValue() {
    // Verify deadband is a reasonable value
    assertTrue(OI.DEADBAND > 0.0);
    assertTrue(OI.DEADBAND < 0.2); // Typical deadband is less than 0.2
  }

  @Test
  void testAxisLimits() {
    // Verify axis limits are correct
    assertEquals(1.0, OI.AxisLimits.MAX_AXIS_LIMIT, 0.001);
    assertEquals(-1.0, OI.AxisLimits.MIN_AXIS_LIMIT, 0.001);
  }

  @Test
  void testHasDriverInputTriggerExists() {
    // Verify hasDriverInput trigger is not null
    assertNotNull(OI.hasDriverInput);
  }

  @Test
  void testButtonTriggersExist() {
    // Verify all button triggers are not null
    assertNotNull(OI.resetGyroButton);
    assertNotNull(OI.retractClimber);
    assertNotNull(OI.extendClimber);
    assertNotNull(OI.intakeOut);
    assertNotNull(OI.intakeIn);
    assertNotNull(OI.intake);
    assertNotNull(OI.shoot);
    assertNotNull(OI.rotateTurretRight);
    assertNotNull(OI.rotateTurretLeft);
    assertNotNull(OI.turretAimPass);
    assertNotNull(OI.turretAimShoot);
  }

  @Test
  void testTurretAxisReturnsValidRange() {
    // Test that getTurretAxis returns a value (even if it's 0 in test environment)
    double axis = OI.getTurretAxis();
    assertTrue(
        axis >= OI.AxisLimits.MIN_AXIS_LIMIT && axis <= OI.AxisLimits.MAX_AXIS_LIMIT,
        "Turret axis should be within valid range");
  }

  @Test
  void testAxisMethodsReturnValues() {
    // These methods should return values without throwing exceptions
    assertDoesNotThrow(() -> OI.getTranslateForwardAxis());
    assertDoesNotThrow(() -> OI.getTranslateStrafeAxis());
    assertDoesNotThrow(() -> OI.getRotationAxis());
    assertDoesNotThrow(() -> OI.getTurtleSpeedTrigger());
    assertDoesNotThrow(() -> OI.getTurboSpeedTrigger());
    assertDoesNotThrow(() -> OI.getTurretAxis());
  }

  @Test
  void testAxisMethodsReturnValidRange() {
    // In test environment, these should return 0 but be within valid range
    double translateForward = OI.getTranslateForwardAxis();
    double translateStrafe = OI.getTranslateStrafeAxis();
    double rotation = OI.getRotationAxis();
    double turtleSpeed = OI.getTurtleSpeedTrigger();
    double turboSpeed = OI.getTurboSpeedTrigger();

    assertTrue(translateForward >= -1.0 && translateForward <= 1.0);
    assertTrue(translateStrafe >= -1.0 && translateStrafe <= 1.0);
    assertTrue(rotation >= -1.0 && rotation <= 1.0);
    assertTrue(turtleSpeed >= 0.0 && turtleSpeed <= 1.0);
    assertTrue(turboSpeed >= 0.0 && turboSpeed <= 1.0);
  }
}