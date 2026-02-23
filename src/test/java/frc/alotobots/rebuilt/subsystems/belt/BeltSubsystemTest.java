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
package frc.alotobots.rebuilt.subsystems.belt;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.belt.io.BeltIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BeltSubsystemTest {

  private BeltIO mockIO;
  private BeltSubsystem subsystem;

  @BeforeEach
  void setUp() {
    mockIO = mock(BeltIO.class);
    subsystem = new BeltSubsystem(mockIO);
  }

  @Test
  void testRunBeltToTargetVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(50);
    subsystem.runBeltToTargetVelocity(velocity);

    verify(mockIO).setBeltVelocity(any(AngularVelocity.class));
  }

  @Test
  void testRunBeltToTargetVelocityWithinLimits() {
    AngularVelocity velocity = RadiansPerSecond.of(50);
    subsystem.runBeltToTargetVelocity(velocity);

    verify(mockIO).setBeltVelocity(argThat(v -> v.in(RadiansPerSecond) <= 100));
  }

  @Test
  void testRunBeltToTargetVelocityClampedAtMax() {
    // Test that velocity above max is clamped
    AngularVelocity highVelocity =
        RadiansPerSecond.of(BeltConstants.Limits.MAX_SPEED.in(RadiansPerSecond) * 2);
    subsystem.runBeltToTargetVelocity(highVelocity);

    verify(mockIO)
        .setBeltVelocity(
            argThat(
                v -> v.in(RadiansPerSecond) <= BeltConstants.Limits.MAX_SPEED.in(RadiansPerSecond)));
  }

  @Test
  void testRunBeltToTargetVelocityClampedAtMin() {
    // Test that velocity below min is clamped
    AngularVelocity lowVelocity =
        RadiansPerSecond.of(-BeltConstants.Limits.MAX_SPEED.in(RadiansPerSecond) * 2);
    subsystem.runBeltToTargetVelocity(lowVelocity);

    verify(mockIO)
        .setBeltVelocity(
            argThat(
                v ->
                    v.in(RadiansPerSecond) >= -BeltConstants.Limits.MAX_SPEED.in(RadiansPerSecond)));
  }

  @Test
  void testRunBeltPercentOutput() {
    subsystem.runBeltPercentOutput(0.5);

    verify(mockIO).setBeltOpenLoop(0.5);
  }

  @Test
  void testRunBeltPercentOutputClampedAtMax() {
    subsystem.runBeltPercentOutput(1.5);

    verify(mockIO)
        .setBeltOpenLoop(argThat(v -> v <= BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testRunBeltPercentOutputClampedAtMin() {
    subsystem.runBeltPercentOutput(-1.5);

    verify(mockIO)
        .setBeltOpenLoop(argThat(v -> v >= -BeltConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testStop() {
    subsystem.stop();

    verify(mockIO).stop();
  }

  @Test
  void testPeriodicCallsUpdateInputs() {
    subsystem.periodic();

    verify(mockIO).updateInputs(any());
  }

  @Test
  void testNegativeVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(-30);
    subsystem.runBeltToTargetVelocity(velocity);

    verify(mockIO).setBeltVelocity(any(AngularVelocity.class));
  }

  @Test
  void testZeroVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(0);
    subsystem.runBeltToTargetVelocity(velocity);

    verify(mockIO).setBeltVelocity(velocity);
  }
}