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
package frc.alotobots.rebuilt.subsystems.intake.extendo;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIO;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIOInputsAutoLogged;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeExtendoSubsystemTest {

  private IntakeExtendoIO mockIO;
  private IntakeExtendoSubsystem subsystem;

  @BeforeEach
  void setUp() {
    mockIO = mock(IntakeExtendoIO.class);
    subsystem = new IntakeExtendoSubsystem(mockIO);
  }

  @Test
  void testRunToTargetPosition() {
    Distance position = Meters.of(0.5);
    subsystem.runToTargetPosition(position);

    verify(mockIO).setIntakeExtendoPosition(any(Distance.class), any());
  }

  @Test
  void testRunToTargetPositionClampedAtMax() {
    Distance highPosition =
        Meters.of(IntakeExtendoConstants.Limits.MAX_EXTENSION.in(Meters) * 2);
    subsystem.runToTargetPosition(highPosition);

    verify(mockIO)
        .setIntakeExtendoPosition(
            argThat(
                d ->
                    d.in(Meters)
                        <= IntakeExtendoConstants.Limits.MAX_EXTENSION.in(Meters)),
            any());
  }

  @Test
  void testRunToTargetPositionClampedAtMin() {
    Distance lowPosition = Meters.of(-1.0);
    subsystem.runToTargetPosition(lowPosition);

    verify(mockIO)
        .setIntakeExtendoPosition(
            argThat(
                d ->
                    d.in(Meters)
                        >= IntakeExtendoConstants.Limits.MIN_EXTENSION.in(Meters)),
            any());
  }

  @Test
  void testRunToTargetVelocity() {
    LinearVelocity velocity = MetersPerSecond.of(0.5);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setIntakeExtendoVelocity(any(LinearVelocity.class));
  }

  @Test
  void testRunToTargetVelocityClampedAtMax() {
    LinearVelocity highVelocity =
        MetersPerSecond.of(IntakeExtendoConstants.Limits.MAX_OPERATOR_VELOCITY.in(MetersPerSecond) * 2);
    subsystem.runToTargetVelocity(highVelocity);

    verify(mockIO)
        .setIntakeExtendoVelocity(
            argThat(
                v ->
                    v.in(MetersPerSecond)
                        <= IntakeExtendoConstants.Limits.MAX_OPERATOR_VELOCITY.in(
                            MetersPerSecond)));
  }

  @Test
  void testRunAtPercentOutput() {
    subsystem.runAtPercentOutput(0.5);

    verify(mockIO).setIntakeExtendoOpenLoop(0.5);
  }

  @Test
  void testRunAtPercentOutputClampedAtMax() {
    subsystem.runAtPercentOutput(1.5);

    verify(mockIO)
        .setIntakeExtendoOpenLoop(
            argThat(v -> v <= IntakeExtendoConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testRunAtPercentOutputClampedAtMin() {
    subsystem.runAtPercentOutput(-1.5);

    verify(mockIO)
        .setIntakeExtendoOpenLoop(
            argThat(v -> v >= -IntakeExtendoConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
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
  void testGetCurrentExtension() {
    IntakeExtendoIOInputsAutoLogged inputs = new IntakeExtendoIOInputsAutoLogged();
    inputs.intakeExtendoDistance = Meters.of(0.3);

    when(mockIO.updateInputs(any()))
        .thenAnswer(
            invocation -> {
              IntakeExtendoIOInputsAutoLogged arg = invocation.getArgument(0);
              arg.intakeExtendoDistance = inputs.intakeExtendoDistance;
              return null;
            });

    subsystem.periodic();
    Distance extension = subsystem.getCurrentExtension();

    assertNotNull(extension);
  }

  @Test
  void testIsAtTargetExtensionInitiallyFalse() {
    assertFalse(subsystem.isAtTargetExtension());
  }

  @Test
  void testZeroVelocity() {
    LinearVelocity velocity = MetersPerSecond.of(0);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setIntakeExtendoVelocity(velocity);
  }

  @Test
  void testNegativeVelocity() {
    LinearVelocity velocity = MetersPerSecond.of(-0.3);
    subsystem.runToTargetVelocity(velocity);

    verify(mockIO).setIntakeExtendoVelocity(any(LinearVelocity.class));
  }
}