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
package frc.alotobots.library.subsystems.bling.constants;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.library.subsystems.bling.io.BlingIO;

/** Constants for LED control and configuration. */
public class BlingConstants {
  /** Maximum brightness setting for LEDs (0.0-1.0) */
  public static final double MAX_LED_BRIGHTNESS = .25;

  /** Total number of LEDs in the strip */
  public static final int NUM_LEDS = 45;

  /** Offset for LED positioning (onboard LEDs are 0-7, external strip starts at 8) */
  public static final int LED_OFFSET = 8;

  /** End index for the external LED strip */
  public static final int LED_END_INDEX = LED_OFFSET + NUM_LEDS - 1;

  /** Duration of non state based LED notifications */
  public static final Time BLING_NOTIFICATION_TIME = Seconds.of(2);

  /** The animation slot used for all bling animations */
  public static final int ANIMATION_SLOT = 0;

  /** Pre-configured animation patterns. */
  public static final class Animations {
    /** Animation pattern for no alliance selected state */
    public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION =
        new ColorFlowAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withDirection(AnimationDirectionValue.Forward);

    public static final ColorFlowAnimation HUB_ACTIVE_ANIMATION_BLUE =
        new ColorFlowAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withColor(new RGBWColor(0, 0, 255))
            .withFrameRate(Hertz.of(50));

    public static final ColorFlowAnimation HUB_ACTIVE_ANIMATION_RED =
        new ColorFlowAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withColor(new RGBWColor(255, 0, 0))
            .withFrameRate(Hertz.of(50));

    public static final LarsonAnimation HUB_APPROACHING_INACTIVE_ANIMATION_BLUE =
        new LarsonAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withSize(25)
            .withFrameRate(Hertz.of(50))
            .withColor(new RGBWColor(0, 0, 255));

    public static final LarsonAnimation HUB_APPROACHING_INACTIVE_ANIMATION_RED =
        new LarsonAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withSize(25)
            .withFrameRate(Hertz.of(50))
            .withColor(new RGBWColor(255, 0, 0));

    public static final StrobeAnimation HUB_APPROACHING_ACTIVE_ANIMATION_BLUE =
        new StrobeAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withFrameRate(Hertz.of(50))
            .withColor(new RGBWColor(0, 0, 255));

    public static final StrobeAnimation HUB_APPROACHING_ACTIVE_ANIMATION_RED =
        new StrobeAnimation(LED_OFFSET, LED_END_INDEX)
            .withSlot(ANIMATION_SLOT)
            .withFrameRate(Hertz.of(50))
            .withColor(new RGBWColor(255, 0, 0));
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
