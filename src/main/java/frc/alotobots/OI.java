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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Operator Interface (OI) class handles all driver control inputs and button mappings. This
 * class manages three Xbox controllers:
 *
 * <ul>
 *   <li>Driver Controller: Primary robot movement and speed control
 *   <li>Co-Driver Controller: State-based controls and shared subsystem control
 *   <li>Co-Driver Backup Controller: Manual subsystem controls and redundant options
 * </ul>
 *
 * The class provides static methods to access controller inputs and defines button bindings for
 * commanding various robot subsystems and states.
 */
public class OI {
  /**
   * The minimum value that joystick inputs must exceed to be registered. This deadband prevents
   * unintended movement from controller drift and provides a stable neutral position for the
   * controls.
   */
  public static final double DEADBAND = 0.05;

  /** Controller port ID for the primary driver's Xbox controller. */
  private static final int DRIVER_CONTROLLER_ID = 0;

  /** Controller port ID for the co-driver's primary Xbox controller. */
  private static final int CO_DRIVER_CONTROLLER_ID = 1;

  private static final int TEST_CONTROLLER_ID = 2;
  private static final int DATA_COLLECTION_CONTROLLER_ID = 3;

  /** Xbox controller instance for the primary driver's control functions. */
  private static final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_ID);

  /** Xbox controller instance for the co-driver's state-based and shared controls. */
  private static final CommandXboxController codriverController =
      new CommandXboxController(CO_DRIVER_CONTROLLER_ID);

  private static final CommandXboxController testController =
      new CommandXboxController(TEST_CONTROLLER_ID);
  private static final CommandXboxController dataController =
      new CommandXboxController(DATA_COLLECTION_CONTROLLER_ID);

  /**
   * Trigger that activates when the driver is using the chassis control sticks. Combines X/Y
   * translation and rotation inputs with deadband application to detect intentional driver input.
   */
  public static final Trigger hasDriverInput =
      new Trigger(
              () ->
                  MathUtil.applyDeadband(driverController.getLeftX(), DEADBAND) != 0
                      || MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND) != 0
                      || MathUtil.applyDeadband(driverController.getRightX(), DEADBAND) != 0)
          .or(
              () ->
                  MathUtil.applyDeadband(dataController.getLeftX(), DEADBAND) != 0
                      || MathUtil.applyDeadband(dataController.getLeftY(), DEADBAND) != 0
                      || MathUtil.applyDeadband(dataController.getRightX(), DEADBAND) != 0);

  /**
   * @return Value between -1.0 (backward) and 1.0 (forward)
   */
  public static double getTranslateForwardAxis() {
    return MathUtil.clamp(driverController.getLeftY() + dataController.getLeftY(), -1.0, 1.0);
  }

  /**
   * @return Value between -1.0 (left) and 1.0 (right)
   */
  public static double getTranslateStrafeAxis() {
    return MathUtil.clamp(driverController.getLeftX() + dataController.getLeftX(), -1.0, 1.0);
  }

  /**
   * @return Value between -1.0 (counter-clockwise) and 1.0 (clockwise)
   */
  public static double getRotationAxis() {
    return MathUtil.clamp(driverController.getRightX() + dataController.getRightX(), -1.0, 1.0);
  }

  /**
   * @return Value between 0.0 (not pressed) and 1.0 (fully pressed)
   */
  public static double getTurtleSpeedTrigger() {
    return MathUtil.clamp(
        driverController.getLeftTriggerAxis() + dataController.getLeftTriggerAxis(), 0.0, 1.0);
  }

  /**
   * @return Value between 0.0 (not pressed) and 1.0 (fully pressed)
   */
  public static double getTurboSpeedTrigger() {
    return driverController.getRightTriggerAxis();
  }

  /* State-based play control triggers */

  // DRIVER CONTROLLER --------------------------------------------------
  public static final Trigger resetGyroButton = driverController.start();
  public static final Trigger intake = driverController.leftTrigger();
  public static final Trigger dumpBalls = driverController.back();
  public static final Trigger shoot = driverController.rightStick();
  public static final Trigger intakeOut = driverController.povUp();
  public static final Trigger intakeIn = driverController.povDown();
  public static final Trigger intakeRollersToggle = driverController.leftStick();
  public static final Trigger toggleClimber = driverController.y();
  public static final Trigger lockWheels = driverController.x();
  public static final Trigger intakeOutFull = driverController.a();
  // TODO MAKE THIS
  public static final Trigger zoneAutoTarget = driverController.b();

  /** Turret Auto Aim for passing */
  public static final Trigger turretAimPass = driverController.leftBumper();

  /** Turret Auto Aim for shooting */
  public static final Trigger turretAimShoot = driverController.rightBumper();

  public static void rumbleDriverController(double percent) {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, percent);
  }

  // CO DRIVER CONTEROLLER --------------------------------------------
  public static final Trigger runKickerAndBeltManual = codriverController.leftBumper();
  public static final Trigger deflectorUpManual = codriverController.povUp();
  public static final Trigger deflectorDownManual = codriverController.povDown();
  public static final Trigger shooterSpeedDownManual = codriverController.povLeft();
  public static final Trigger shooterSpeedUpManual = codriverController.povRight();
  public static final Trigger runKickerAndBeltOutManual = codriverController.a();
  public static final Trigger putDeflectorDown = codriverController.b();

  /**
   * @return Value between -1.0 (up) and 1.0 (down)
   */
  public static double getClimberManualAxis() {
    return MathUtil.applyDeadband(codriverController.getLeftY(), DEADBAND);
  }

  /**
   * @return Value between -1.0 (counter-clockwise) and 1.0 (clockwise)
   */
  public static double getTurretManualAxis() {
    return MathUtil.applyDeadband(codriverController.getRightX(), DEADBAND);
  }

  /**
   * @return Value between 0 (off) and 1.0 (100%)
   */
  public static double getShooterManualAxis() {
    return MathUtil.applyDeadband(codriverController.getRightTriggerAxis(), DEADBAND);
  }

  // TEST CONTROLLER --------------------------------------------------
  // FOR WHATEVER YOU WANT
  public static final Trigger sysIDQuasistaticFwd = testController.a();
  public static final Trigger sysIDQuasistaticRev = testController.b();
  public static final Trigger sysIDDynamicFwd = testController.x();
  public static final Trigger sysIDDynamicRev = testController.y();

  public static final Trigger testButton = testController.a();
  public static final Trigger testButton2 = testController.b();

  // DATA CONTROLLER --------------------------------------------------
  public static final Trigger shootData = dataController.rightStick();
  public static final Trigger logData = dataController.back();
  public static final Trigger rpmDownData = dataController.x();
  public static final Trigger rpmUpData = dataController.b();
  public static final Trigger deflectorDownData = dataController.y();
  public static final Trigger deflectorUpData = dataController.a();
  public static final Trigger intakeOutData = dataController.povUp();
  public static final Trigger intakeInData = dataController.povDown();

  /**
   * Gets the manual turret control input, Applies deadband.
   *
   * @return Value between -.2 (left) and .2 (right)
   */
  public static double getTestingTurretAxis() {
    double primary = testController.getRightX();
    return MathUtil.applyDeadband(primary, DEADBAND) * .2;
  }

  public static double getTestingClimberAxis() {
    double primary = testController.getLeftY();
    return MathUtil.applyDeadband(primary, DEADBAND) * .7;
  }

  public static double getTestingIntakeAxis() {
    double primary = testController.getRightY();
    return MathUtil.applyDeadband(primary, DEADBAND) * .2;
  }

  /** Contains defined limits for controller axis inputs. */
  public static final class AxisLimits {
    public static final double MAX_AXIS_LIMIT = 1.0;
    public static final double MIN_AXIS_LIMIT = -1.0;
  }
}
