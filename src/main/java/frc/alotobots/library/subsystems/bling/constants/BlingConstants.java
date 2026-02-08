/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.bling.constants;

import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.BLUE_ALLIANCE_COLOR;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.NO_ALLIANCE_COLOR;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.RED_ALLIANCE_COLOR;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.units.measure.Time;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOReal;

/** Constants for LED control and configuration. */
public class BlingConstants {
  /** Maximum brightness setting for LEDs (0.0-1.0) */
  public static final double MAX_LED_BRIGHTNESS = .25;

  /** Total number of LEDs in the strip */
  public static final int NUM_LEDS = 92;

  /** Offset for LED positioning */
  public static final int LED_OFFSET = 8;

  /** Status LED state configuration */
  public static final boolean DISABLE_STATUS_LED = false;

  /** LED strip type configuration */
  public static final StripTypeValue LED_TYPE = StripTypeValue.GRB;

  /** Duration of non state based LED notifications */
  public static final Time BLING_NOTIFICATION_TIME = Seconds.of(2);

  /** Pre-configured animation patterns. */
  public static final class Animations {
    /** Animation pattern for no alliance selected state */
    public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION =
        new ColorFlowAnimation(
            LED_OFFSET, NUM_LEDS).withColor(BlingIO.LoggedColorToRGBW(NO_ALLIANCE_COLOR));


    /** Count down timer for endgame - Red slowly disappearing (30s) */
    public static final LarsonAnimation ENDGAME_COUNTDOWN_RED_ANIMATION =
        new LarsonAnimation(
            LED_OFFSET, NUM_LEDS).withColor(BlingIO.LoggedColorToRGBW(RED_ALLIANCE_COLOR));

    /** Count down timer for endgame - Blue slowly disappearing (30s) */
    public static final LarsonAnimation ENDGAME_COUNTDOWN_BLUE_ANIMATION =
        new LarsonAnimation(
            LED_OFFSET, NUM_LEDS).withColor(BlingIO.LoggedColorToRGBW(BLUE_ALLIANCE_COLOR));

    /** Time to climb animation (triggers like 5s before end of match) - Blink red rapidly (2s) */
    public static final StrobeAnimation TIME_TO_CLIMB_RED_ANIMATION =
        new StrobeAnimation(NUM_LEDS, LED_OFFSET).withColor(BlingIO.LoggedColorToRGBW(RED_ALLIANCE_COLOR));
    /** Time to climb animation (triggers like 5s before end of match) - Blink blue rapidly (2s) */
    public static final StrobeAnimation TIME_TO_CLIMB_BLUE_ANIMATION =
        new StrobeAnimation(NUM_LEDS, LED_OFFSET).withColor(BlingIO.LoggedColorToRGBW(BLUE_ALLIANCE_COLOR));

    /** Auto driving - Rainbow (While active) */
    public static final RainbowAnimation AUTO_DRIVING_ANIMATION =
        new RainbowAnimation(NUM_LEDS, LED_OFFSET);
  }

  /** Pre-defined color configurations. */
  public static final class Colors {
    /** Color setting for LEDs off */
    public static final BlingIO.LoggedColor OFF_COLOR = new BlingIO.LoggedColor(0, 0, 0);

    /** Color setting for blue alliance */
    public static final BlingIO.LoggedColor BLUE_ALLIANCE_COLOR =
        new BlingIO.LoggedColor(0, 0, 255);

    /** Color setting for red alliance */
    public static final BlingIO.LoggedColor RED_ALLIANCE_COLOR = new BlingIO.LoggedColor(255, 0, 0);

    /** Cage Color */
    public static final BlingIO.LoggedColor CAGE_SWITCH_COLOR = new BlingIO.LoggedColor(0, 255, 0);

    /** Color setting for no alliance selected */
    public static final BlingIO.LoggedColor NO_ALLIANCE_COLOR =
        new BlingIO.LoggedColor(255, 255, 0);

    /** Color setting for intake occupied state */
    public static final BlingIO.LoggedColor INTAKE_OCCUPIED_COLOR =
        new BlingIO.LoggedColor(0, 255, 0);

    /** Color setting for shooter occupied state */
    public static final BlingIO.LoggedColor SHOOTER_OCCUPIED_COLOR =
        new BlingIO.LoggedColor(140, 48, 255);

    /** Color setting for shooter ready state */
    public static final BlingIO.LoggedColor SHOOTER_READY_COLOR =
        new BlingIO.LoggedColor(255, 145, 0);
  }
}
