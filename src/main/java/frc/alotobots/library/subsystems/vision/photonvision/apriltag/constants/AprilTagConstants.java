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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.UtilityClass;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("resource")
@UtilityClass
public class AprilTagConstants {
  // AprilTag layout
  public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // OFFSETS
  // FORWARD: +, LEFT: +, UP: + (USED FOR APRILTAGS)
  private static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Left
        new Transform3d(
            new Translation3d(0.34, 0.24, 0.495),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(-15))),
        // Front (Back) Right NEEDS CALIBRATION
        new Transform3d(
            new Translation3d(-0.276225, -0.3048, 0.19),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(-80.6))),
        // Back Left
        new Transform3d(
            new Translation3d(-0.305, 0.29, 0.19),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(73.5))),
        // Back Right
        new Transform3d(
            new Translation3d(-0.335, -0.245, 0.19),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-20), Math.toRadians(155)))
      };

  // CAMERAS
  public static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig("FL_AprilTag", CAMERA_OFFSETS[0], new SimCameraProperties()),
    new CameraConfig("FR_AprilTag", CAMERA_OFFSETS[1], new SimCameraProperties()),
    new CameraConfig("BL_AprilTag", CAMERA_OFFSETS[2], new SimCameraProperties()),
    new CameraConfig("BR_AprilTag", CAMERA_OFFSETS[3], new SimCameraProperties())
  };

  /*
   * The standard deviations to use for the vision
   */
  // NEEDS TO BE COPIED FROM PHOTONVISION ON COMPETITION DAY!!
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  // LOWER = TRUST MORE
  public static double MULTI_TAG_LINEAR_STD_DEV_BASE = 0.06; // Meters
  public static double MULTI_TAG_ANGULAR_STD_DEV_BASE =
      0.174533; // 10deg (not used in main estimator, only secondary)

  public static double SINGLE_TAG_LINEAR_STD_DEV_BASE = 0.12; // Meters
  public static double SINGLE_TAG_ANGULAR_STD_DEV_BASE =
      1000; // DO NOT USE FOR HEADING CORRECTION! MAKES LOOP!!!

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // SHOULD NEVER BE LESS THAN 1.0, NUMBERS GREATER THAN 1 = TRUST LESS
  public static double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0, 1.0, 1.2};

  // Basic Filtering
  public static double MULTITAG_MAX_AMBIGUITY = 0.3;
  public static double MAX_Z_ERROR = 0.75;

  // Constants for single tag processing
  public static final double SINGLE_TAG_MAX_AMBIGUITY = 0.2;
  public static final double SINGLE_TAG_MAX_DISTANCE = 2.0; // meters

  /** Time after which a pose is considered stale (seconds) */
  public static final double POSE_TIMEOUT = 0.06;
}
