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
package frc.alotobots.rebuilt.subsystems.belt.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DefaultBeltRunAtVelocityTest {

  private BeltSubsystem mockSubsystem;
  private Supplier<AngularVelocity> velocitySupplier;
  private DefaultBeltRunAtVelocity command;

  @BeforeEach
  void setUp() {
    mockSubsystem = mock(BeltSubsystem.class);
    velocitySupplier = () -> RadiansPerSecond.of(50);
    command = new DefaultBeltRunAtVelocity(mockSubsystem, velocitySupplier);
  }

  @Test
  void testCommandRequiresSubsystem() {
    assertTrue(command.getRequirements().contains(mockSubsystem));
  }

  @Test
  void testExecuteCallsSubsystemWithVelocity() {
    command.execute();

    verify(mockSubsystem).runBeltToTargetVelocity(any(AngularVelocity.class));
  }

  @Test
  void testExecuteUsesSuppliedVelocity() {
    AngularVelocity testVelocity = RadiansPerSecond.of(75);
    Supplier<AngularVelocity> testSupplier = () -> testVelocity;
    DefaultBeltRunAtVelocity testCommand =
        new DefaultBeltRunAtVelocity(mockSubsystem, testSupplier);

    testCommand.execute();

    verify(mockSubsystem).runBeltToTargetVelocity(testVelocity);
  }

  @Test
  void testEndCallsStop() {
    command.end(false);

    verify(mockSubsystem).stop();
  }

  @Test
  void testEndCallsStopWhenInterrupted() {
    command.end(true);

    verify(mockSubsystem).stop();
  }

  @Test
  void testIsFinishedReturnsFalse() {
    assertFalse(command.isFinished());
  }

  @Test
  void testInitializeDoesNotThrow() {
    assertDoesNotThrow(() -> command.initialize());
  }

  @Test
  void testMultipleExecuteCalls() {
    command.execute();
    command.execute();
    command.execute();

    verify(mockSubsystem, times(3)).runBeltToTargetVelocity(any(AngularVelocity.class));
  }
}