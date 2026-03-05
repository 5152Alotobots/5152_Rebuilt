// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.alotobots.rebuilt.util;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;

import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import lombok.Setter;

import static edu.wpi.first.units.Units.Seconds;

/**
 * @author 6328 Mechanical Advantage
 * Adapted by 5152 Alotobots
 * */
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
      ShiftEnum currentShift, Time elapsedTime, Time remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final Time[] shiftStartTimes = {Seconds.of(0.0), Seconds.of(10.0), Seconds.of(35.0), Seconds.of(60.0), Seconds.of(85.0), Seconds.of(110.0)};
  private static final Time[] shiftEndTimes = {Seconds.of(10.0), Seconds.of(35.0), Seconds.of(60.0), Seconds.of(85.0), Seconds.of(110.0), Seconds.of(140.0)};

  private static final Time minFuelCountDelay = Seconds.of(1.0);
  private static final Time maxFuelCountDelay = Seconds.of(2.0);
  private static final Time shiftEndFuelCountExtension = Seconds.of(3.0);
  private static final Time minTimeOfFlight = LaunchCalculator.getMinFuelTimeOfFlight();
  private static final Time maxTimeOfFlight = LaunchCalculator.getMaxFuelTimeOfFlight();
  private static final Time approachingActiveFudge =
          minTimeOfFlight.plus(minFuelCountDelay).times(-1);
  private static final Time endingActiveFudge =
          shiftEndFuelCountExtension.plus(maxTimeOfFlight.plus(maxFuelCountDelay).times(-1));

  public static final Time autoEndTime = Seconds.of(20.0);
  public static final Time teleopDuration = Seconds.of(140.0);
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

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
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, Time[] shiftStartTimes, Time[] shiftEndTimes) {
    Time currentTime = Seconds.of(shiftTimer.get());
    Time stateTimeElapsed = Seconds.of(shiftTimer.get());
    Time stateTimeRemaining = Seconds.of(0.0);
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime.minus(currentTime);
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
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  private static final Time[] STARTING_ACTIVE_START_TIMES = new Time[] {
          Seconds.of(0.0),
          Seconds.of(10.0),
          Seconds.of(35.0).plus(endingActiveFudge),
          Seconds.of(60.0).plus(approachingActiveFudge),
          Seconds.of(85.0).plus(endingActiveFudge),
          Seconds.of(110.0).plus(approachingActiveFudge)
  };
  private static final Time[] STARTING_ACTIVE_END_TIMES = new Time[] {
          Seconds.of(10.0),
          Seconds.of(35.0).plus(endingActiveFudge),
          Seconds.of(60.0).plus(approachingActiveFudge),
          Seconds.of(85.0).plus(endingActiveFudge),
          Seconds.of(110.0).plus(approachingActiveFudge),
          Seconds.of(140.0)
  };
  private static final Time[] STARTING_INACTIVE_START_TIMES = new Time[] {
          Seconds.of(0.0),
          Seconds.of(10.0).plus(endingActiveFudge),
          Seconds.of(35.0).plus(approachingActiveFudge),
          Seconds.of(60.0).plus(endingActiveFudge),
          Seconds.of(85.0).plus(approachingActiveFudge),
          Seconds.of(110.0)
  };
  private static final Time[] STARTING_INACTIVE_END_TIMES = new Time[] {
          Seconds.of(10.0).plus(endingActiveFudge),
          Seconds.of(35.0).plus(approachingActiveFudge),
          Seconds.of(60.0).plus(endingActiveFudge),
          Seconds.of(85.0).plus(approachingActiveFudge),
          Seconds.of(110.0),
          Seconds.of(140.0)
  };

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    if (shiftSchedule[1]) {
      return getShiftInfo(shiftSchedule, STARTING_ACTIVE_START_TIMES, STARTING_ACTIVE_END_TIMES);
    }
    return getShiftInfo(shiftSchedule, STARTING_INACTIVE_START_TIMES, STARTING_INACTIVE_END_TIMES);
  }
}