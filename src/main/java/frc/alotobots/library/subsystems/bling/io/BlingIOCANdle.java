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
package frc.alotobots.library.subsystems.bling.io;

import static frc.alotobots.Constants.CanId.CANDLE_CAN_ID;
import static frc.alotobots.Constants.CanId.RIO_CAN_BUS;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.*;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.OFF_COLOR;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

/**
 * Hardware implementation of the BlingIO interface for controlling physical LED strips. Uses CTRE's
 * Phoenix 6 CANdle device for LED control.
 */
public class BlingIOCANdle implements BlingIO {
  /** CANdle controller for LED management */
  private final CANdle candle;

  /** Current solid color setting */
  private LoggedColor currentColor;

  /** Current animation setting */
  private ControlRequest currentAnimation;

  /**
   * Constructs a new BlingIOCANdle instance. Initializes the CANdle controller with Phoenix 6
   * configuration.
   */
  public BlingIOCANdle() {
    this.candle = new CANdle(CANDLE_CAN_ID, RIO_CAN_BUS);

    var CANdleConfig = new CANdleConfiguration();
    CANdleConfig.LED.StripType = StripTypeValue.GRB;
    CANdleConfig.LED.BrightnessScalar = MAX_LED_BRIGHTNESS;
    CANdleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    CANdleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;

    candle.getConfigurator().apply(CANdleConfig);

    // Clear all animation slots on startup
    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
  }

  @Override
  public void updateInputs(BlingIOInputs inputs) {
    inputs.currentSolidColor = currentColor;
    if (currentAnimation != null) {
      inputs.animationName = currentAnimation.getClass().getSimpleName();
    } else {
      inputs.animationName = "";
    }
    inputs.hasAnimation = currentAnimation != null;
    inputs.hasColor = currentColor != null;
  }

  @Override
  public void setAnimation(ControlRequest animation) {
    currentAnimation = animation;
    candle.setControl(animation);
  }

  @Override
  public void clearAnimation() {
    currentAnimation = null;
    candle.setControl(new EmptyAnimation(ANIMATION_SLOT));
  }

  @Override
  public void setSolidColor(LoggedColor color, int from, int to) {
    currentColor = color;
    candle.setControl(
        new SolidColor(from, from + to - 1)
            .withColor(new RGBWColor(color.red(), color.green(), color.blue(), 0)));
  }

  @Override
  public void clearSolidColor(int from, int to) {
    currentColor = OFF_COLOR;
    candle.setControl(new SolidColor(from, from + to - 1).withColor(new RGBWColor(0, 0, 0, 0)));
  }
}
