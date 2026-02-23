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
package frc.alotobots.rebuilt.subsystems.intake.extendo.constants;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class IntakeExtendoTalonFXConstantsTest {

  @Test
  void testVelocityPIDConstants() {
    // Verify velocity PID constants are non-negative
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KP >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KI >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KD >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KG >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KS >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.VelocityPIDConstants.KV >= 0);
  }

  @Test
  void testPositionPIDConstants() {
    // Verify position PID constants are non-negative
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KP >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KI >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KD >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KA >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KG >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KS >= 0);
    assertTrue(IntakeExtendoTalonFXConstants.PIDConstants.PositionPIDConstants.KV >= 0);
  }

  @Test
  void testMotionMagicConstants() {
    // Verify motion magic constants are positive
    assertTrue(
        IntakeExtendoTalonFXConstants.MotionMagicConstants.CRUISE_VELOCITY.in(MetersPerSecond) > 0);
    assertTrue(
        IntakeExtendoTalonFXConstants.MotionMagicConstants.ACCELERATION.in(
                MetersPerSecondPerSecond)
            > 0);
    assertTrue(IntakeExtendoTalonFXConstants.MotionMagicConstants.JERK >= 0);
  }

  @Test
  void testMotorSafetyLimits() {
    // Verify motor safety limits are positive and reasonable
    assertTrue(
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps) > 0);
    assertTrue(
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps) < 0);
    assertTrue(IntakeExtendoTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps) > 0);

    // Verify limits are reasonable (typical for FRC motors)
    assertTrue(
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps) < 100);
    assertTrue(
        IntakeExtendoTalonFXConstants.MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps) > -100);
    assertTrue(IntakeExtendoTalonFXConstants.MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps) < 100);
  }

  @Test
  void testNeutralModeNotNull() {
    assertNotNull(IntakeExtendoTalonFXConstants.MECHANISM_NEUTRAL_MODE);
  }

  @Test
  void testMotorDirectionNotNull() {
    assertNotNull(IntakeExtendoTalonFXConstants.MOTOR_DIRECTION);
  }

  @Test
  void testExtensionPerRotationPositive() {
    // Extension per rotation should be positive
    assertTrue(IntakeExtendoTalonFXConstants.EXTENSION_PER_ROTATION > 0);
  }

  @Test
  void testExtensionPerRotationReasonable() {
    // Extension per rotation should be a reasonable value (in meters)
    // Typical values would be between 0.001 and 0.1 meters per rotation
    assertTrue(IntakeExtendoTalonFXConstants.EXTENSION_PER_ROTATION > 0.001);
    assertTrue(IntakeExtendoTalonFXConstants.EXTENSION_PER_ROTATION < 0.1);
  }

  @Test
  void testCruiseVelocityReasonable() {
    // Cruise velocity should be reasonable for an intake extendo
    double cruiseVelocity =
        IntakeExtendoTalonFXConstants.MotionMagicConstants.CRUISE_VELOCITY.in(MetersPerSecond);
    assertTrue(cruiseVelocity > 0.1); // At least 0.1 m/s
    assertTrue(cruiseVelocity < 10.0); // Less than 10 m/s
  }
}