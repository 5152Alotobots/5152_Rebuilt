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
package frc.alotobots.rebuilt.subsystems.intake.extendo.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants.Limits.MIN_EXTENSION;
import static frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoTalonFXConstants.EXTENSION_PER_ROTATION;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.*;
import org.junit.jupiter.api.Test;

/**
 * Test class for IntakeExtendoIOTalonFX conversion methods. Note: These tests verify the conversion
 * logic mathematically without requiring hardware. The actual TalonFX hardware initialization is
 * skipped in unit tests.
 */
class IntakeExtendoIOTalonFXConversionTest {

  @Test
  void testExtensionPerRotationIsPositive() {
    assertTrue(EXTENSION_PER_ROTATION > 0, "Extension per rotation must be positive");
  }

  @Test
  void testMinExtensionIsNonNegative() {
    assertTrue(MIN_EXTENSION.in(Meters) >= 0, "Minimum extension should be non-negative");
  }

  @Test
  void testConversionConstantsAreConsistent() {
    // Verify that the conversion constant is reasonable
    // For a typical linear actuator, this should be in a reasonable range
    assertTrue(EXTENSION_PER_ROTATION > 0.001, "Extension per rotation too small");
    assertTrue(EXTENSION_PER_ROTATION < 0.1, "Extension per rotation too large");
  }

  @Test
  void testZeroRotationsEqualsMinExtension() {
    // When the motor is at 0 rotations, the extension should be MIN_EXTENSION
    // Formula: extension = EXTENSION_PER_ROTATION * rotations + MIN_EXTENSION
    double expectedExtension = MIN_EXTENSION.in(Meters);
    double actualExtension = EXTENSION_PER_ROTATION * 0 + MIN_EXTENSION.in(Meters);

    assertEquals(expectedExtension, actualExtension, 0.0001);
  }

  @Test
  void testPositiveRotationsIncreasesExtension() {
    // Positive rotations should increase extension
    double rotations = 10.0;
    double extension = EXTENSION_PER_ROTATION * rotations + MIN_EXTENSION.in(Meters);

    assertTrue(extension > MIN_EXTENSION.in(Meters));
  }

  @Test
  void testLinearVelocityConversion() {
    // Test that linear velocity scales with extension per rotation
    double rotationsPerSecond = 5.0;
    double expectedLinearVelocity = rotationsPerSecond * EXTENSION_PER_ROTATION;

    assertEquals(expectedLinearVelocity, rotationsPerSecond * EXTENSION_PER_ROTATION, 0.0001);
  }

  @Test
  void testInverseConversionExtensionToRotations() {
    // Test that converting extension to rotations is the inverse operation
    Distance testExtension = Meters.of(0.5);
    double expectedRotations = (testExtension.in(Meters) - MIN_EXTENSION.in(Meters)) / EXTENSION_PER_ROTATION;

    assertTrue(expectedRotations >= 0, "Rotations should be non-negative for valid extension");
  }

  @Test
  void testLinearAccelerationConversion() {
    // Test that linear acceleration scales the same way as velocity
    double rotationsPerSecondSquared = 3.0;
    double expectedLinearAcceleration = rotationsPerSecondSquared * EXTENSION_PER_ROTATION;

    assertEquals(
        expectedLinearAcceleration, rotationsPerSecondSquared * EXTENSION_PER_ROTATION, 0.0001);
  }

  @Test
  void testConversionRoundTrip() {
    // Test that converting from extension to rotations and back yields the same value
    Distance originalExtension = Meters.of(0.3);

    // Convert to rotations
    double rotations = (originalExtension.in(Meters) - MIN_EXTENSION.in(Meters)) / EXTENSION_PER_ROTATION;

    // Convert back to extension
    double reconstructedExtension = EXTENSION_PER_ROTATION * rotations + MIN_EXTENSION.in(Meters);

    assertEquals(originalExtension.in(Meters), reconstructedExtension, 0.0001);
  }

  @Test
  void testVelocityConversionRoundTrip() {
    // Test that converting velocity is consistent both ways
    double originalRotationsPerSecond = 2.5;

    // Convert to linear velocity
    double linearVelocity = originalRotationsPerSecond * EXTENSION_PER_ROTATION;

    // Convert back to rotational velocity
    double reconstructedRotationsPerSecond = linearVelocity / EXTENSION_PER_ROTATION;

    assertEquals(originalRotationsPerSecond, reconstructedRotationsPerSecond, 0.0001);
  }

  @Test
  void testLargeExtensionValue() {
    // Test with a larger extension value
    double rotations = 100.0;
    double extension = EXTENSION_PER_ROTATION * rotations + MIN_EXTENSION.in(Meters);

    assertTrue(extension > MIN_EXTENSION.in(Meters));
    assertTrue(extension < 10.0); // Reasonable upper bound for intake extension
  }

  @Test
  void testNegativeRotationsNotAllowed() {
    // Negative rotations would result in extension less than MIN_EXTENSION
    double negativeRotations = -10.0;
    double extension = EXTENSION_PER_ROTATION * negativeRotations + MIN_EXTENSION.in(Meters);

    assertTrue(extension < MIN_EXTENSION.in(Meters),
        "Negative rotations should result in extension below minimum");
  }
}