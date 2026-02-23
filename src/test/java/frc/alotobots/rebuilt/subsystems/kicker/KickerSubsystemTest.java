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
package frc.alotobots.rebuilt.subsystems.kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class KickerSubsystemTest {

  private KickerIO mockIO;
  private KickerSubsystem subsystem;

  @BeforeEach
  void setUp() {
    mockIO = mock(KickerIO.class);
    subsystem = new KickerSubsystem(mockIO);
  }

  @Test
  void testRunToTargetVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(50);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setKickerVelocity(any(AngularVelocity.class));
  }

  @Test
  void testRunToTargetVelocityClampedAtMax() {
    AngularVelocity highVelocity =
        RadiansPerSecond.of(KickerConstants.Limits.MAX_SPEED.in(RadiansPerSecond) * 2);
    subsystem.runToTargetVelocity(highVelocity);

    verify(mockIO)
        .setKickerVelocity(
            argThat(
                v ->
                    v.in(RadiansPerSecond)
                        <= KickerConstants.Limits.MAX_SPEED.in(RadiansPerSecond)));
  }

  @Test
  void testRunToTargetVelocityClampedAtMin() {
    AngularVelocity lowVelocity =
        RadiansPerSecond.of(-KickerConstants.Limits.MAX_SPEED.in(RadiansPerSecond) * 2);
    subsystem.runToTargetVelocity(lowVelocity);

    verify(mockIO)
        .setKickerVelocity(
            argThat(
                v ->
                    v.in(RadiansPerSecond)
                        >= -KickerConstants.Limits.MAX_SPEED.in(RadiansPerSecond)));
  }

  @Test
  void testRunAtPercentOutput() {
    subsystem.runAtPercentOutput(0.5);

    verify(mockIO).setKickerOpenLoop(0.5);
  }

  @Test
  void testRunAtPercentOutputClampedAtMax() {
    subsystem.runAtPercentOutput(1.5);

    verify(mockIO)
        .setKickerOpenLoop(argThat(v -> v <= KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testRunAtPercentOutputClampedAtMin() {
    subsystem.runAtPercentOutput(-1.5);

    verify(mockIO)
        .setKickerOpenLoop(argThat(v -> v >= -KickerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
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
  void testZeroVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(0);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setKickerVelocity(velocity);
  }

  @Test
  void testNegativeVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(-30);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setKickerVelocity(any(AngularVelocity.class));
  }
}