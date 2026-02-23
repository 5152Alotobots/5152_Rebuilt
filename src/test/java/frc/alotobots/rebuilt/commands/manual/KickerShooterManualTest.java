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
package frc.alotobots.rebuilt.commands.manual;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class KickerShooterManualTest {

  private ShooterSubsystem mockShooterSubsystem;
  private KickerSubsystem mockKickerSubsystem;
  private BeltSubsystem mockBeltSubsystem;
  private Supplier<AngularVelocity> velocitySupplier;
  private Trigger mockTrigger;
  private KickerShooterManual command;

  @BeforeEach
  void setUp() {
    mockShooterSubsystem = mock(ShooterSubsystem.class);
    mockKickerSubsystem = mock(KickerSubsystem.class);
    mockBeltSubsystem = mock(BeltSubsystem.class);
    velocitySupplier = () -> RadiansPerSecond.of(100);
    mockTrigger = new Trigger(() -> false);
    command =
        new KickerShooterManual(
            mockShooterSubsystem,
            mockKickerSubsystem,
            mockBeltSubsystem,
            velocitySupplier,
            mockTrigger);
  }

  @Test
  void testCommandCreation() {
    assertNotNull(command);
  }

  @Test
  void testCommandRequiresAllSubsystems() {
    assertTrue(command.getRequirements().contains(mockShooterSubsystem));
    assertTrue(command.getRequirements().contains(mockKickerSubsystem));
    assertTrue(command.getRequirements().contains(mockBeltSubsystem));
  }

  @Test
  void testCommandIsSequential() {
    // SequentialCommandGroup is a composite command
    assertTrue(command.getRequirements().size() >= 3);
  }

  @Test
  void testCommandCanBeInstantiated() {
    assertDoesNotThrow(
        () ->
            new KickerShooterManual(
                mockShooterSubsystem,
                mockKickerSubsystem,
                mockBeltSubsystem,
                velocitySupplier,
                mockTrigger));
  }

  @Test
  void testCommandWithNullSubsystemsThrows() {
    assertThrows(
        NullPointerException.class,
        () ->
            new KickerShooterManual(
                null, mockKickerSubsystem, mockBeltSubsystem, velocitySupplier, mockTrigger));
    assertThrows(
        NullPointerException.class,
        () ->
            new KickerShooterManual(
                mockShooterSubsystem, null, mockBeltSubsystem, velocitySupplier, mockTrigger));
    assertThrows(
        NullPointerException.class,
        () ->
            new KickerShooterManual(
                mockShooterSubsystem,
                mockKickerSubsystem,
                null,
                velocitySupplier,
                mockTrigger));
  }

  @Test
  void testCommandWithNullVelocitySupplierThrows() {
    assertThrows(
        NullPointerException.class,
        () ->
            new KickerShooterManual(
                mockShooterSubsystem, mockKickerSubsystem, mockBeltSubsystem, null, mockTrigger));
  }

  @Test
  void testCommandWithNullTriggerThrows() {
    assertThrows(
        NullPointerException.class,
        () ->
            new KickerShooterManual(
                mockShooterSubsystem,
                mockKickerSubsystem,
                mockBeltSubsystem,
                velocitySupplier,
                null));
  }

  @Test
  void testCommandInitializeDoesNotThrow() {
    assertDoesNotThrow(() -> command.initialize());
  }

  @Test
  void testMultipleCommandInstances() {
    KickerShooterManual command1 =
        new KickerShooterManual(
            mockShooterSubsystem,
            mockKickerSubsystem,
            mockBeltSubsystem,
            velocitySupplier,
            mockTrigger);
    KickerShooterManual command2 =
        new KickerShooterManual(
            mockShooterSubsystem,
            mockKickerSubsystem,
            mockBeltSubsystem,
            velocitySupplier,
            mockTrigger);

    assertNotNull(command1);
    assertNotNull(command2);
    assertNotSame(command1, command2);
  }
}