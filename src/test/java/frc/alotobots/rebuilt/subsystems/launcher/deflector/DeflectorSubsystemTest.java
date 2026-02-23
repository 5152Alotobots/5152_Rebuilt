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
package frc.alotobots.rebuilt.subsystems.launcher.deflector;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIOInputsAutoLogged;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DeflectorSubsystemTest {

  private DeflectorIO mockIO;
  private DeflectorSubsystem subsystem;

  @BeforeEach
  void setUp() {
    mockIO = mock(DeflectorIO.class);
    subsystem = new DeflectorSubsystem(mockIO);
  }

  @Test
  void testRunToTargetAngle() {
    Angle angle = Radians.of(0.5);
    subsystem.runToTargetAngle(angle);

    verify(mockIO).setDeflectorPosition(any(Angle.class), any());
  }

  @Test
  void testRunToTargetAngleClampedAtMax() {
    Angle highAngle = Radians.of(DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE.in(Radians) * 2);
    subsystem.runToTargetAngle(highAngle);

    verify(mockIO)
        .setDeflectorPosition(
            argThat(
                a ->
                    a.in(Radians)
                        <= DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE.in(Radians)),
            any());
  }

  @Test
  void testRunToTargetAngleClampedAtMin() {
    Angle lowAngle = Radians.of(-5.0);
    subsystem.runToTargetAngle(lowAngle);

    verify(mockIO)
        .setDeflectorPosition(
            argThat(
                a ->
                    a.in(Radians)
                        >= DeflectorConstants.Limits.DEFLECTOR_MIN_ANGLE.in(Radians)),
            any());
  }

  @Test
  void testRunToTargetVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(0.5);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setDeflectorVelocity(any(AngularVelocity.class));
  }

  @Test
  void testRunToTargetVelocityClampedAtMax() {
    AngularVelocity highVelocity =
        RadiansPerSecond.of(DeflectorConstants.Limits.DEFLECTOR_MAX_VELOCITY.in(RadiansPerSecond) * 2);
    subsystem.runToTargetVelocity(highVelocity);

    verify(mockIO)
        .setDeflectorVelocity(
            argThat(
                v ->
                    v.in(RadiansPerSecond)
                        <= DeflectorConstants.Limits.DEFLECTOR_MAX_VELOCITY.in(
                            RadiansPerSecond)));
  }

  @Test
  void testRunAtPercentOutput() {
    subsystem.runAtPercentOutput(0.5);

    verify(mockIO).setDeflectorOpenLoop(0.5);
  }

  @Test
  void testRunAtPercentOutputClampedAtMax() {
    subsystem.runAtPercentOutput(1.5);

    verify(mockIO)
        .setDeflectorOpenLoop(
            argThat(v -> v <= DeflectorConstants.Limits.DEFLECTOR_MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testRunAtPercentOutputClampedAtMin() {
    subsystem.runAtPercentOutput(-1.5);

    verify(mockIO)
        .setDeflectorOpenLoop(
            argThat(v -> v >= -DeflectorConstants.Limits.DEFLECTOR_MAX_OPEN_LOOP_PERCENTAGE));
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
  void testGetCurrentAngle() {
    DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();
    inputs.deflectorAngle = Radians.of(0.3);

    when(mockIO.updateInputs(any()))
        .thenAnswer(
            invocation -> {
              DeflectorIOInputsAutoLogged arg = invocation.getArgument(0);
              arg.deflectorAngle = inputs.deflectorAngle;
              return null;
            });

    subsystem.periodic();
    Angle angle = subsystem.getCurrentAngle();

    assertNotNull(angle);
  }

  @Test
  void testIsAtTargetAngleInitiallyFalse() {
    assertFalse(subsystem.isAtTargetAngle());
  }

  @Test
  void testZeroAngle() {
    Angle angle = Radians.of(0);
    subsystem.runToTargetAngle(angle);

    verify(mockIO).setDeflectorPosition(any(Angle.class), any());
  }

  @Test
  void testNegativeVelocity() {
    AngularVelocity velocity = RadiansPerSecond.of(-0.3);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setDeflectorVelocity(any(AngularVelocity.class));
  }
}