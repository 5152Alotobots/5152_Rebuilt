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
package frc.alotobots.rebuilt.util.hubshift;

import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.rebuilt.util.hubshift.HubShiftUtilConstants.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

/**
 * @author 6328 Mechanical Advantage Adapted by 5152 Alotobots
 */
public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift,
      Time elapsedTime,
      Time remainingTime,
      boolean active,
      boolean approachingActive,
      boolean approachingInactive) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (winOverride.isPresent()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? ACTIVE_SCHEDULE
            : INACTIVE_SCHEDULE;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, Time[] shiftStartTimes, Time[] shiftEndTimes) {
    Time currentTime = Seconds.of(shiftTimer.get());
    Time stateTimeElapsed = Seconds.of(shiftTimer.get());
    Time stateTimeRemaining = Seconds.of(0.0);
    boolean active = false;
    boolean approachingActive = false;
    boolean approachingInactive = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = AUTO_END_TIME.minus(currentTime);
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime.gte(shiftStartTimes[i]) && currentTime.lt(shiftEndTimes[i])) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime.minus(shiftStartTimes[currentShiftIndex]);
      stateTimeRemaining = shiftEndTimes[currentShiftIndex].minus(currentTime);

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime.minus(shiftStartTimes[currentShiftIndex - 1]);
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1].minus(currentTime);
        }
      }

      active = currentSchedule[currentShiftIndex];
      approachingActive =
          (!currentSchedule[currentShiftIndex]
              && stateTimeRemaining.lte(APPROACHING_ACTIVE_NOTIFICAITON_TIME));
      approachingInactive =
          (currentSchedule[currentShiftIndex]
              && stateTimeRemaining.lte(APPROACHING_INACTIVE_NOTIFICATION_TIME));
      currentShift = shiftsEnums[currentShiftIndex];
    }
    return new ShiftInfo(
        currentShift,
        stateTimeElapsed,
        stateTimeRemaining,
        active,
        approachingActive,
        approachingInactive);
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), SHIFT_START_TIMES, SHIFT_END_TIMES);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    if (shiftSchedule[1]) {
      return getShiftInfo(shiftSchedule, STARTING_ACTIVE_START_TIMES, STARTING_ACTIVE_END_TIMES);
    }
    return getShiftInfo(shiftSchedule, STARTING_INACTIVE_START_TIMES, STARTING_INACTIVE_END_TIMES);
  }
}
