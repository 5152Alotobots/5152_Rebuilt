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
package frc.alotobots;

import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.OI.*;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.BLING_NOTIFICATION_TIME;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.commands.*;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIORealCompetition;
import frc.alotobots.library.subsystems.bling.io.BlingIOSim;
import frc.alotobots.library.subsystems.bling.util.BlingUtil;
import frc.alotobots.library.subsystems.swervedrive.*;
import frc.alotobots.library.subsystems.swervedrive.commands.*;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.*;
import frc.alotobots.library.subsystems.vision.questnav.QuestNavSubsystem;
import frc.alotobots.library.subsystems.vision.questnav.io.*;
import frc.alotobots.util.NotificationPresets;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final QuestNavSubsystem questNavSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private final AutoNamedCommands autoNamedCommands;
  private LoggedDashboardChooser<Command> autoChooser;
  private SwerveDriveSimulation driveSimulation;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot hardware initialization

        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_RIGHT.index));
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands = new AutoNamedCommands(swerveDriveSubsystem);
        configureAutoChooser();
        questNavSubsystem =
            new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new QuestNavIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getRotation),
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getRotation));
        blingSubsystem = new BlingSubsystem(new BlingIORealCompetition());
        break;

      case SIM:
        Pose2d simStartPose = new Pose2d(3, 3, new Rotation2d(0));
        driveSimulation =
            new SwerveDriveSimulation(
                Constants.tunerConstants.getDriveTrainSimulationConfig(), simStartPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // Simulation hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIOTalonFXSim(
                    ModulePosition.FRONT_LEFT.index,
                    driveSimulation.getModules()[ModulePosition.FRONT_LEFT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.FRONT_RIGHT.index,
                    driveSimulation.getModules()[ModulePosition.FRONT_RIGHT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.BACK_LEFT.index,
                    driveSimulation.getModules()[ModulePosition.BACK_LEFT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.BACK_RIGHT.index,
                    driveSimulation.getModules()[ModulePosition.BACK_RIGHT.index]));
        swerveDriveSubsystem.setPose(simStartPose);
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands = new AutoNamedCommands(swerveDriveSubsystem);
        configureAutoChooser();

        questNavSubsystem =
            new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new QuestNavIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getPose),
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getPose));

        blingSubsystem = new BlingSubsystem(new BlingIOSim());
        break;

      default:
        // Replay mode initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands = new AutoNamedCommands(swerveDriveSubsystem);
        configureAutoChooser();

        questNavSubsystem =
            new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new QuestNavIO() {});
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIO() {});
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Commands that run when nothing else is */
  private void configureDefaultCommands() {
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
  }

  /** Contains button based commands */
  private void configureLogicCommands() {
    lockWheelsButton.onTrue(new InstantCommand(swerveDriveSubsystem::stopWithX));
    // TEMPORARY!!
    resetGyroButton.onTrue(
        new InstantCommand(() -> swerveDriveSubsystem.setPose(new Pose2d(0, 0, Rotation2d.kZero))));
    // Bling
    BlingUtil.scheduleAtMatchTime(
        new BlingEndgameCountdown(blingSubsystem)
            .withTimeout(20)
            .andThen(new BlingTimeToClimb(blingSubsystem).withTimeout(BLING_NOTIFICATION_TIME)),
        Seconds.of(30));
  }

  private void configureAutoChooser() {
    // Set up auto routines
    // Initialize the chooser and register it with the dashboard
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add listener for auto path changes
    autoChooser.getSendableChooser().onChange(this::handleAutoPathChange);

    // Add SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new WheelRadiusCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization", new FeedforwardCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void handleAutoPathChange(String autoName) {
    pathPlannerManager
        .getAutoStartPose(autoName)
        .ifPresent(
            pose -> {
              swerveDriveSubsystem.setPose(pose);
              questNavSubsystem.resetPose(pose);
              NotificationPresets.Auto.sendAutoPathChangeNotification(autoName);
            });
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
