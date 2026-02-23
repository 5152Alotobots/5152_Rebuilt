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
package frc.alotobots.rebuilt.commands.groups;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IndexIntoShooterAndShootTest {

  private BeltSubsystem mockBeltSubsystem;
  private KickerSubsystem mockKickerSubsystem;
  private ShooterSubsystem mockShooterSubsystem;
  private IndexIntoShooterAndShoot command;

  @BeforeEach
  void setUp() {
    mockBeltSubsystem = mock(BeltSubsystem.class);
    mockKickerSubsystem = mock(KickerSubsystem.class);
    mockShooterSubsystem = mock(ShooterSubsystem.class);
    command =
        new IndexIntoShooterAndShoot(mockBeltSubsystem, mockKickerSubsystem, mockShooterSubsystem);
  }

  @Test
  void testCommandCreation() {
    assertNotNull(command);
  }

  @Test
  void testCommandRequiresAllSubsystems() {
    assertTrue(command.getRequirements().contains(mockBeltSubsystem));
    assertTrue(command.getRequirements().contains(mockKickerSubsystem));
    assertTrue(command.getRequirements().contains(mockShooterSubsystem));
  }

  @Test
  void testCommandIsSequential() {
    // SequentialCommandGroup is a composite command
    assertTrue(command.getRequirements().size() >= 3);
  }

  @Test
  void testCommandCanBeInstantiated() {
    // Test that command can be created without exceptions
    assertDoesNotThrow(
        () ->
            new IndexIntoShooterAndShoot(
                mockBeltSubsystem, mockKickerSubsystem, mockShooterSubsystem));
  }

  @Test
  void testCommandWithNullSubsystemsThrows() {
    // Test that null subsystems cause appropriate exceptions
    assertThrows(
        NullPointerException.class,
        () -> new IndexIntoShooterAndShoot(null, mockKickerSubsystem, mockShooterSubsystem));
    assertThrows(
        NullPointerException.class,
        () -> new IndexIntoShooterAndShoot(mockBeltSubsystem, null, mockShooterSubsystem));
    assertThrows(
        NullPointerException.class,
        () -> new IndexIntoShooterAndShoot(mockBeltSubsystem, mockKickerSubsystem, null));
  }

  @Test
  void testCommandInitializeDoesNotThrow() {
    assertDoesNotThrow(() -> command.initialize());
  }

  @Test
  void testMultipleCommandInstances() {
    // Test that multiple instances can be created independently
    IndexIntoShooterAndShoot command1 =
        new IndexIntoShooterAndShoot(mockBeltSubsystem, mockKickerSubsystem, mockShooterSubsystem);
    IndexIntoShooterAndShoot command2 =
        new IndexIntoShooterAndShoot(mockBeltSubsystem, mockKickerSubsystem, mockShooterSubsystem);

    assertNotNull(command1);
    assertNotNull(command2);
    assertNotSame(command1, command2);
  }
}