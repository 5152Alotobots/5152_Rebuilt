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
package frc.alotobots.library.subsystems.bling.io;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.alotobots.Constants.CanId;
import frc.alotobots.library.subsystems.bling.constants.BlingConstants;

public class BlingIOReal implements BlingIO {
  private final CANdle candle;
  private LoggedColor currentColor;

  // CHANGED: Use ControlRequest, as there is no specific "Animation" base class in V6
  private ControlRequest currentAnimation;

  public BlingIOReal() {
    this.candle = new CANdle(CanId.CANDLE_CAN_ID, "rio");

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.BrightnessScalar = BlingConstants.MAX_LED_BRIGHTNESS;
    config.LED.StripType = StripTypeValue.GRB;

    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(BlingIOInputs inputs) {
    inputs.currentSolidColor = currentColor;

    if (currentAnimation != null) {
      // .toString() or .getClass().getSimpleName() still works on ControlRequest
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
    candle.setControl(currentAnimation);
  }

  @Override
  public void clearAnimation() {
    currentAnimation = null;
    // To "clear", we effectively stop the animation control request.
    // Setting LEDs to 0 (Black) via setLEDs usually overrides the control request
    // or you can send an empty/off control request if one exists.
    // For now, explicitly turning them off is the safest "clear" state.
    candle.setControl(new EmptyAnimation(0));
  }

  @Override
  public void setSolidColor(LoggedColor color, int startIdx, int count) {
    // If an animation is running, you might need to nullify it
    // depending on if you want this to override the animation.
    currentColor = color;

    clearAnimation();
    currentAnimation =
        new SolidColor(startIdx, startIdx + count)
            .withColor(new RGBWColor(color.red(), color.green(), color.blue()));

    setAnimation(currentAnimation);
  }

  @Override
  public void clearSolidColor(int startIdx, int count) {
    clearAnimation();
  }
}
