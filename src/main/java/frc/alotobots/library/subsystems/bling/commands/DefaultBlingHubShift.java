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
package frc.alotobots.library.subsystems.bling.commands;

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Animations.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.rebuilt.util.hubshift.HubShiftUtil;

public class DefaultBlingHubShift extends Command {
  private final BlingSubsystem blingSubsystem;
  private DriverStation.Alliance alliance;

  public DefaultBlingHubShift(BlingSubsystem blingSubsystem) {
    this.blingSubsystem = blingSubsystem;
    addRequirements(blingSubsystem);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isEmpty()) {
      DriverStation.reportWarning(
          "Could not get Alliance for DefaultBlingHubShift command!", false);
      end(true);
    } else {
      alliance = DriverStation.getAlliance().get();
    }
  }

  @Override
  public void execute() {
    // Close to inactive
    if (HubShiftUtil.getOfficialShiftInfo().approachingInactive()) {
      blingSubsystem.setAnimation(
          alliance == DriverStation.Alliance.Blue
              ? HUB_APPROACHING_INACTIVE_ANIMATION_BLUE
              : HUB_APPROACHING_INACTIVE_ANIMATION_RED);
    } else if (HubShiftUtil.getOfficialShiftInfo().approachingActive()) {
      blingSubsystem.setAnimation(
          alliance == DriverStation.Alliance.Blue
              ? HUB_APPROACHING_ACTIVE_ANIMATION_BLUE
              : HUB_APPROACHING_ACTIVE_ANIMATION_RED);
    } else {
      if (HubShiftUtil.getOfficialShiftInfo().active()) {
        blingSubsystem.setAnimation(
            alliance == DriverStation.Alliance.Blue
                ? HUB_ACTIVE_ANIMATION_BLUE
                : HUB_ACTIVE_ANIMATION_RED);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    blingSubsystem.clear();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
