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
package frc.alotobots.rebuilt.subsystems.kicker.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DefaultKickerRunAtVelocityTest {

  private KickerSubsystem mockSubsystem;
  private Supplier<AngularVelocity> velocitySupplier;
  private DefaultKickerRunAtVelocity command;

  @BeforeEach
  void setUp() {
    mockSubsystem = mock(KickerSubsystem.class);
    velocitySupplier = () -> RadiansPerSecond.of(50);
    command = new DefaultKickerRunAtVelocity(mockSubsystem, velocitySupplier);
  }

  @Test
  void testCommandRequiresSubsystem() {
    assertTrue(command.getRequirements().contains(mockSubsystem));
  }

  @Test
  void testExecuteCallsSubsystemWithVelocity() {
    command.execute();

    verify(mockSubsystem).runToTargetVelocity(any(AngularVelocity.class));
  }

  @Test
  void testExecuteUsesSuppliedVelocity() {
    AngularVelocity testVelocity = RadiansPerSecond.of(75);
    Supplier<AngularVelocity> testSupplier = () -> testVelocity;
    DefaultKickerRunAtVelocity testCommand =
        new DefaultKickerRunAtVelocity(mockSubsystem, testSupplier);

    testCommand.execute();

    verify(mockSubsystem).runToTargetVelocity(testVelocity);
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

    verify(mockSubsystem, times(3)).runToTargetVelocity(any(AngularVelocity.class));
  }

  @Test
  void testDynamicVelocityChange() {
    // Test that the command uses a fresh velocity value each execute
    AngularVelocity[] velocities = {
      RadiansPerSecond.of(10), RadiansPerSecond.of(20), RadiansPerSecond.of(30)
    };
    int[] index = {0};
    Supplier<AngularVelocity> dynamicSupplier = () -> velocities[index[0]++];
    DefaultKickerRunAtVelocity dynamicCommand =
        new DefaultKickerRunAtVelocity(mockSubsystem, dynamicSupplier);

    dynamicCommand.execute();
    verify(mockSubsystem).runToTargetVelocity(velocities[0]);

    dynamicCommand.execute();
    verify(mockSubsystem).runToTargetVelocity(velocities[1]);

    dynamicCommand.execute();
    verify(mockSubsystem).runToTargetVelocity(velocities[2]);
  }
}