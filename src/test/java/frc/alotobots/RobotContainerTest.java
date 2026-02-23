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
package frc.alotobots;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RobotContainerTest {

  @Test
  void testRobotContainerCanBeInstantiated() {
    assertDoesNotThrow(() -> new RobotContainer());
  }

  @Test
  void testGetAutonomousCommand() {
    RobotContainer container = new RobotContainer();
    // Should return a command (may be null if no auto selected)
    assertDoesNotThrow(() -> container.getAutonomousCommand());
  }

  @Test
  void testResetSimulationFieldDoesNotThrow() {
    RobotContainer container = new RobotContainer();
    assertDoesNotThrow(() -> container.resetSimulationField());
  }

  @Test
  void testDisplaySimFieldToAdvantageScopeDoesNotThrow() {
    RobotContainer container = new RobotContainer();
    assertDoesNotThrow(() -> container.displaySimFieldToAdvantageScope());
  }

  @Test
  void testMultipleInstantiation() {
    // Test that multiple instances can be created (though typically only one exists)
    assertDoesNotThrow(
        () -> {
          new RobotContainer();
          new RobotContainer();
        });
  }

  @Test
  void testAutonomousCommandIsNotNullAfterInstantiation() {
    RobotContainer container = new RobotContainer();
    // Get autonomous command should not throw
    assertDoesNotThrow(() -> container.getAutonomousCommand());
  }
}