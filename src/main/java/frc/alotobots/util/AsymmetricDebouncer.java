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
package frc.alotobots.util;

import edu.wpi.first.math.filter.Debouncer;

public class AsymmetricDebouncer {
  private final Debouncer rising, falling;
  private boolean output = false;

  public AsymmetricDebouncer(double risingSec, double fallingSec) {
    rising = new Debouncer(risingSec, Debouncer.DebounceType.kRising);
    falling = new Debouncer(fallingSec, Debouncer.DebounceType.kFalling);
  }

  public boolean calculate(boolean input) {
    output = output ? falling.calculate(input) : rising.calculate(input);
    return output;
  }
}
