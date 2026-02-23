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
package frc.alotobots.rebuilt.subsystems.intake.roller;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeRollerSubsystemTest {

  private IntakeRollerIO mockIO;
  private IntakeRollerSubsystem subsystem;

  @BeforeEach
  void setUp() {
    mockIO = mock(IntakeRollerIO.class);
    subsystem = new IntakeRollerSubsystem(mockIO);
  }

  @Test
  void testRunAtPercentOutput() {
    subsystem.runAtPercentOutput(0.5);

    verify(mockIO).setIntakeRollerOpenLoop(0.5);
  }

  @Test
  void testRunAtPercentOutputClampedAtMax() {
    subsystem.runAtPercentOutput(1.5);

    verify(mockIO)
        .setIntakeRollerOpenLoop(
            argThat(v -> v <= IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
  }

  @Test
  void testRunAtPercentOutputClampedAtMin() {
    subsystem.runAtPercentOutput(-1.5);

    verify(mockIO)
        .setIntakeRollerOpenLoop(
            argThat(v -> v >= -IntakeRollerConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE));
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
  void testZeroPercentOutput() {
    subsystem.runAtPercentOutput(0.0);

    verify(mockIO).setIntakeRollerOpenLoop(0.0);
  }

  @Test
  void testNegativePercentOutput() {
    subsystem.runAtPercentOutput(-0.5);

    verify(mockIO).setIntakeRollerOpenLoop(-0.5);
  }

  @Test
  void testFullForwardOutput() {
    subsystem.runAtPercentOutput(1.0);

    verify(mockIO).setIntakeRollerOpenLoop(1.0);
  }

  @Test
  void testFullReverseOutput() {
    subsystem.runAtPercentOutput(-1.0);

    verify(mockIO).setIntakeRollerOpenLoop(-1.0);
  }
}