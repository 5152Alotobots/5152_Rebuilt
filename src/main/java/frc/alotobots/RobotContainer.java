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

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.OI.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOReal;
import frc.alotobots.library.subsystems.bling.io.BlingIOSim;
import frc.alotobots.library.subsystems.swervedrive.*;
import frc.alotobots.library.subsystems.swervedrive.commands.*;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.*;
import frc.alotobots.rebuilt.commands.groups.DeployIntakeAndIntake;
import frc.alotobots.rebuilt.commands.groups.IndexIntoShooterAndShoot;
import frc.alotobots.rebuilt.commands.groups.LauncherTargetHub;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.io.BeltIO;
import frc.alotobots.rebuilt.subsystems.belt.io.BeltIOTalonFX;
import frc.alotobots.rebuilt.subsystems.climber.ClimberSubsystem;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIO;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIOTalonFX;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.extendo.commands.DefaultIntakeExtendoRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.intake.extendo.commands.IntakeExtendoRunToExtension;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIO;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIOTalonFX;
import frc.alotobots.rebuilt.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIO;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIOTalonFX;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIO;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIOTalonFX;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorRunToPosition;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIOVortex;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIO;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOSim;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOTalonFX;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.DefaultTurretRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretRunPercentOut;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIO;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIOSim;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIOTalonFXS;
import frc.alotobots.util.NotificationPresets;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  //   private final QuestNavSubsystem questNavSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private final AutoNamedCommands autoNamedCommands;
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final BeltSubsystem beltSubsystem;
  private final DeflectorSubsystem deflectorSubsystem;
  private final IntakeExtendoSubsystem intakeExtendoSubsystem;
  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final LaunchCalculator launchCalculator;
  private final ClimberSubsystem climberSubsystem;
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
        // questNavSubsystem =
        //     new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new
        // QuestNavIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getRotation),
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getRotation));
        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
        blingSubsystem = new BlingSubsystem(new BlingIOReal());
        turretSubsystem = new TurretSubsystem(new TurretIOTalonFXS());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        beltSubsystem = new BeltSubsystem(new BeltIOTalonFX());
        kickerSubsystem = new KickerSubsystem(new KickerIOTalonFX());
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIOTalonFX());
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIOTalonFX());
        deflectorSubsystem = new DeflectorSubsystem(new DeflectorIOVortex());
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);
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
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        configureAutoChooser();

        // questNavSubsystem =
        //    new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new
        // QuestNavIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});

        deflectorSubsystem = new DeflectorSubsystem(new DeflectorIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIOSim());
        turretSubsystem = new TurretSubsystem(new TurretIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
        beltSubsystem = new BeltSubsystem(new BeltIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIO() {});
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIO() {});
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);
        break;

      default:
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        deflectorSubsystem = new DeflectorSubsystem(new DeflectorIO() {});
        autoNamedCommands = new AutoNamedCommands(swerveDriveSubsystem);
        configureAutoChooser();

        // questNavSubsystem =
        //    new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new QuestNavIO()
        // {});
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIO() {});
        shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        beltSubsystem = new BeltSubsystem(new BeltIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        turretSubsystem = new TurretSubsystem(new TurretIO() {});
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIO() {});
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIO() {});
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Commands that run when nothing else is */
  private void configureDefaultCommands() {

    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
    turretSubsystem.setDefaultCommand(
        new DefaultTurretRunAtVelocity(turretSubsystem, OI::getTurretAxis));
    intakeExtendoSubsystem.setDefaultCommand(
        new DefaultIntakeExtendoRunAtVelocity(intakeExtendoSubsystem, OI::getClimberAxis));
    // climberSubsystem.setDefaultCommand(
    // new ClimberRunOpenLoop(climberSubsystem, OI::getClimberAxis));
    // turretSubsystem.setDefaultCommand(
    //     new RunTurretToTarget(
    //         new TurretAngleCalculations(swerveDriveSubsystem, turretSubsystem),
    // turretSubsystem));
    turretSubsystem.setDefaultCommand(new TurretRunPercentOut(turretSubsystem, OI::getTurretAxis));
  }

  // TODO: remove this
  @AutoLogOutput(key = "DataCollection/targetrpm")
  private AngularVelocity shooterVelocity = RotationsPerSecond.of(30);

  /** Contains button based commands */
  private void configureLogicCommands() {

    // lockWheelsButton.onTrue(new InstantCommand(swerveDriveSubsystem::stopWithX));
    // Intake Extendo

    intakeOut.onTrue(
        new DeployIntakeAndIntake(intakeExtendoSubsystem, intakeRollerSubsystem).until(intakeIn));
    intakeIn.onTrue(
        new IntakeExtendoRunToExtension(
            intakeExtendoSubsystem, IntakeExtendoConstants.Setpoints.STOWED));
    // Launcher
    turretAimShoot.whileTrue(
        new LauncherTargetHub(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            launchCalculator));
    shoot.whileTrue(
        new IndexIntoShooterAndShoot(
            beltSubsystem, kickerSubsystem, shooterSubsystem, () -> shooterVelocity));
    logData.onTrue(
        new InstantCommand(launchCalculator::clearLaunchingParameters)
            .andThen(
                new InstantCommand(
                    () ->
                        Logger.recordOutput(
                            "DataCollection/data",
                            String.format(
                                "Turret Deg: %f, Deflector Deg: %f, RPS: %f, Distance: %f",
                                turretSubsystem.getCurrentAngle().in(Degrees),
                                deflectorSubsystem.getCurrentAngle().in(Degrees),
                                shooterVelocity.in(RotationsPerSecond),
                                launchCalculator
                                    .getParameters()
                                    .dataCollectionDebugDistance()
                                    .in(Meters))))));
    deflectorDown.onTrue(
        new InstantCommand(
            () ->
                new DeflectorRunToPosition(
                        deflectorSubsystem,
                        deflectorSubsystem.getCurrentAngle().plus(Degrees.of(5)))
                    .schedule()));
    deflectorUp.onTrue(
        new InstantCommand(
            () ->
                new DeflectorRunToPosition(
                        deflectorSubsystem,
                        deflectorSubsystem.getCurrentAngle().minus(Degrees.of(5)))
                    .schedule()));
    rpmDown.onTrue(
        new InstantCommand(
            () -> shooterVelocity = shooterVelocity.minus(RotationsPerSecond.of(2.5))));
    rpmUp.onTrue(
        new InstantCommand(
            () -> shooterVelocity = shooterVelocity.plus(RotationsPerSecond.of(2.5))));

    // TODO WE NEED RIGHT BUTTONS BUT THIS WORKS
    // testButton.whileTrue(
    //     new ClimberRunToExtension(climberSubsystem,
    // ClimberConstants.Limits.MAX_CLIMB_EXTENSION));
    // testButton2.whileTrue(
    //     new ClimberRunToExtension(climberSubsystem,
    // ClimberConstants.Limits.MIN_CLIMB_EXTENSION));

    // sysIDDynamicFwd.whileTrue(
    //     turretSubsystem.sysIdFwdDynamic().andThen(new InstantCommand(turretSubsystem::stop)));
    // sysIDDynamicRev.whileTrue(
    //     turretSubsystem.sysIdRevDynamic().andThen(new InstantCommand(turretSubsystem::stop)));
    // sysIDQuasistaticFwd.whileTrue(
    //     turretSubsystem.sysIdFwdQuasiStatic().andThen(new
    // InstantCommand(turretSubsystem::stop)));
    // sysIDQuasistaticRev.whileTrue(
    //     turretSubsystem.sysIdRevQuasiStatic().andThen(new
    // InstantCommand(turretSubsystem::stop)));

    // TEMPORARY!!
    resetGyroButton.onTrue(
        new InstantCommand(() -> swerveDriveSubsystem.setPose(new Pose2d(0, 0, Rotation2d.kZero))));
  }

  private void configureAutoChooser() {
    // Set up auto routines
    // Initialize the chooser and register it with the dashboard
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add listener for auto path changes
    autoChooser.getSendableChooser().onChange(this::handleAutoPathChange);

    addSysIdAutos();
  }

  private void addSysIdAutos() {
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
              // questNavSubsystem.resetPose(pose);
              NotificationPresets.Auto.sendAutoPathChangeNotification(autoName);
            });
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, Rotation2d.kZero));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
