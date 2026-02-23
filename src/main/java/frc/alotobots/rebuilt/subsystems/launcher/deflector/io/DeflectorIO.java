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
package frc.alotobots.rebuilt.subsystems.launcher.deflector.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DeflectorIO {
  enum PIDSlots {
    DEFAULT_POSITION,
    VELOCITY,
    OPEN_LOOP
  }

  /** Data structure for inputs from turret hardware. */
  @AutoLog
  public static class DeflectorIOInputs {
    public PIDSlots deflectorMotorPidSlot = PIDSlots.DEFAULT_POSITION;
    
    public boolean deflectorMotorConnected = false;
    public boolean deflectorEncoderConnected = false;
    
    public Angle deflectorMotorAngle = Rotations.zero();
    public Angle deflectorEncoderAngle = Rotations.zero();
    public Angle deflectorAngle = Rotations.zero();
    
    public AngularVelocity deflectorEncoderVelocity = RotationsPerSecond.zero();
    public AngularVelocity deflectorMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration deflectorMotorAcceleration = RotationsPerSecondPerSecond.zero();
    public Voltage deflectorMotorVolts = Volts.zero();
    public Current deflectorMotorCurrent = Amps.zero();
    public boolean backLimit = false;
  }

  /**
   * Updates the Deflector input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  default void updateInputs(DeflectorIOInputs inputs) {}

  /**
   * Sets the Deflector to run to a target position using closed-loop control.
   *
   * @param position The target angle to move to
   * @param pidSlot The PID slot to use
   */
  default void setDeflectorPosition(Angle position, PIDSlots pidSlot) {}

  default void setDeflectorPosition(Angle position) {}

  /**
   * Sets the wrist to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use
   */
  default void setDeflectorVelocity(AngularVelocity velocity, PIDSlots pidSlot) {}

  default void setDeflectorVelocity(AngularVelocity velocity) {}

  /**
   * Runs the Deflector using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  default void setDeflectorOpenLoop(double percentOutput) {}

  /** Stops all Deflector motor movement. */
  default void stop() {}
}
