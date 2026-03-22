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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.alotobots.OI.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.swervedrive.ModulePosition;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.FeedforwardCharacterization;
import frc.alotobots.library.subsystems.swervedrive.commands.WheelRadiusCharacterization;
import frc.alotobots.library.subsystems.swervedrive.io.GyroIO;
import frc.alotobots.library.subsystems.swervedrive.io.GyroIOPigeon2;
import frc.alotobots.library.subsystems.swervedrive.io.ModuleIO;
import frc.alotobots.library.subsystems.swervedrive.io.ModuleIOTalonFXReal;
import frc.alotobots.library.subsystems.swervedrive.io.ModuleIOTalonFXSim;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIO;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOPhotonVision;
import frc.alotobots.rebuilt.commands.groups.*;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.belt.io.BeltIO;
import frc.alotobots.rebuilt.subsystems.belt.io.BeltIOTalonFX;
import frc.alotobots.rebuilt.subsystems.climber.ClimberSubsystem;
import frc.alotobots.rebuilt.subsystems.climber.commands.ClimberRunToExtension;
import frc.alotobots.rebuilt.subsystems.climber.commands.DefaultClimberRunOpenLoop;
import frc.alotobots.rebuilt.subsystems.climber.constants.ClimberConstants;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIO;
import frc.alotobots.rebuilt.subsystems.climber.io.ClimberIOTalonFX;
import frc.alotobots.rebuilt.subsystems.intake.extendo.IntakeExtendoSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.extendo.commands.IntakeExtendoRunToExtension;
import frc.alotobots.rebuilt.subsystems.intake.extendo.constants.IntakeExtendoConstants;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIO;
import frc.alotobots.rebuilt.subsystems.intake.extendo.io.IntakeExtendoIOTalonFX;
import frc.alotobots.rebuilt.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.alotobots.rebuilt.subsystems.intake.roller.commands.IntakeRollerEject;
import frc.alotobots.rebuilt.subsystems.intake.roller.commands.IntakeRollerIntake;
import frc.alotobots.rebuilt.subsystems.intake.roller.constants.IntakeRollerConstants;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIO;
import frc.alotobots.rebuilt.subsystems.intake.roller.io.IntakeRollerIOTalonFX;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIO;
import frc.alotobots.rebuilt.subsystems.kicker.io.KickerIOTalonFX;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorRunToPosition;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIO;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.io.DeflectorIOVortex;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIO;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOSim;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.io.ShooterIOTalonFX;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.DefaultTurretRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIO;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIOSim;
import frc.alotobots.rebuilt.subsystems.launcher.turret.io.TurretIOTalonFXS;
import frc.alotobots.rebuilt.subsystems.roller.RollerSubsystem;
import frc.alotobots.rebuilt.subsystems.roller.io.RollerIO;
import frc.alotobots.rebuilt.subsystems.roller.io.RollerIOSparkMax;
import frc.alotobots.rebuilt.util.hubshift.HubShiftUtil;
import frc.alotobots.util.NotificationPresets;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  //   private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private final RollerSubsystem rollerSubsystem;
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
        // questNavSubsystem =
        //     new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new
        // QuestNavIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getRotation),
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getRotation),
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[2], swerveDriveSubsystem::getRotation));
        climberSubsystem = new ClimberSubsystem(new ClimberIOTalonFX());
        // blingSubsystem = new BlingSubsystem(new BlingIOCANdle());
        turretSubsystem = new TurretSubsystem(new TurretIOTalonFXS());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
        beltSubsystem = new BeltSubsystem(new BeltIOTalonFX());
        kickerSubsystem = new KickerSubsystem(new KickerIOTalonFX());
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIOTalonFX());
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIOTalonFX());
        deflectorSubsystem = new DeflectorSubsystem(new DeflectorIOVortex());
        rollerSubsystem = new RollerSubsystem(new RollerIOSparkMax());
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);

        autoNamedCommands =
            new AutoNamedCommands(
                swerveDriveSubsystem,
                deflectorSubsystem,
                intakeExtendoSubsystem,
                intakeRollerSubsystem,
                turretSubsystem,
                shooterSubsystem,
                kickerSubsystem,
                beltSubsystem,
                rollerSubsystem,
                climberSubsystem,
                launchCalculator);
        configureAutoChooser();
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
        // blingSubsystem = new BlingSubsystem(new BlingIOSim());
        turretSubsystem = new TurretSubsystem(new TurretIOSim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
        beltSubsystem = new BeltSubsystem(new BeltIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIO() {});
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIO() {});
        rollerSubsystem = new RollerSubsystem(new RollerIO() {});
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);
        autoNamedCommands =
            new AutoNamedCommands(
                swerveDriveSubsystem,
                deflectorSubsystem,
                intakeExtendoSubsystem,
                intakeRollerSubsystem,
                turretSubsystem,
                shooterSubsystem,
                kickerSubsystem,
                beltSubsystem,
                rollerSubsystem,
                climberSubsystem,
                launchCalculator);
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
        configureAutoChooser();

        // questNavSubsystem =
        //    new QuestNavSubsystem(swerveDriveSubsystem::addVisionMeasurement, new QuestNavIO()
        // {});
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});
        // blingSubsystem = new BlingSubsystem(new BlingIO() {});
        shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
        kickerSubsystem = new KickerSubsystem(new KickerIO() {});
        beltSubsystem = new BeltSubsystem(new BeltIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        turretSubsystem = new TurretSubsystem(new TurretIO() {});
        intakeExtendoSubsystem = new IntakeExtendoSubsystem(new IntakeExtendoIO() {});
        intakeRollerSubsystem = new IntakeRollerSubsystem(new IntakeRollerIO() {});
        rollerSubsystem = new RollerSubsystem(new RollerIO() {});
        launchCalculator =
            new LaunchCalculator(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::getChassisSpeeds,
                swerveDriveSubsystem::getFieldChassisSpeeds);
        autoNamedCommands =
            new AutoNamedCommands(
                swerveDriveSubsystem,
                deflectorSubsystem,
                intakeExtendoSubsystem,
                intakeRollerSubsystem,
                turretSubsystem,
                shooterSubsystem,
                kickerSubsystem,
                beltSubsystem,
                rollerSubsystem,
                climberSubsystem,
                launchCalculator);
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Commands that run when nothing else is */
  private void configureDefaultCommands() {

    // Swerve
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());

    // Bling
    // blingSubsystem.setDefaultCommand(
    //     new NoAllianceWaiting(blingSubsystem).andThen(new SetToAllianceColor(blingSubsystem)));

    // --- BACKUPS ---
    // Shooter
    shooterSubsystem.setDefaultCommand(
        new DefaultShooterRunAtVelocity(shooterSubsystem, OI::getShooterManualAxis)
            .onlyWhile(() -> OI.getShooterManualAxis() != 0.0));
    // Climber
    climberSubsystem.setDefaultCommand(
        new DefaultClimberRunOpenLoop(climberSubsystem, OI::getClimberManualAxis)
            .onlyWhile(() -> OI.getClimberManualAxis() != 0.0));
    // Turret
    turretSubsystem.setDefaultCommand(
        new DefaultTurretRunAtVelocity(turretSubsystem, OI::getTurretManualAxis)
            .onlyWhile(() -> OI.getTurretManualAxis() != 0.0));
  }

  // DATA COLLECTION ---------
  @AutoLogOutput(key = "DataCollection/shooterVelocityOverride")
  private AngularVelocity shooterVelocityOverride = RotationsPerSecond.of(30);

  @AutoLogOutput(key = "DataCollection/deflectorAngleOverride")
  private Angle deflectorAngleOverride = DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE;

  // --------------------------

  /** Contains button based commands */
  private void configureLogicCommands() {
    // General
    RobotModeTriggers.teleop().onTrue(new InstantCommand(HubShiftUtil::initialize));
    // RobotModeTriggers.teleop().whileTrue(new DefaultBlingHubShift(blingSubsystem));

    // Swerve
    lockWheels.onTrue(new InstantCommand(swerveDriveSubsystem::stopWithX));
    resetGyroButton.onTrue(
        new InstantCommand(
            () ->
                swerveDriveSubsystem.setPose(
                    new Pose2d(
                        swerveDriveSubsystem.getPose().getTranslation(), Rotation2d.kZero))));

    // Intake
    intakeOut.onTrue(
        new IntakeExtendoRunToExtension(
                intakeExtendoSubsystem, IntakeExtendoConstants.Setpoints.DEPLOYED)
            .until(intakeIn));
    intakeIn.onTrue(
        new IntakeExtendoRunToExtension(
            intakeExtendoSubsystem, IntakeExtendoConstants.Setpoints.STOWED));
    intakeRollersToggle.toggleOnTrue(
        new IntakeRollerIntake(
                intakeRollerSubsystem,
                () -> IntakeRollerConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE)
            .alongWith(
                new StartEndCommand(
                    () -> OI.rumbleDriverController(0.02), () -> OI.rumbleDriverController(0.0)))
        // .alongWith(
        //     new StartEndCommand(
        //         () ->
        //             blingSubsystem.setAnimation(
        //                 BlingConstants.Animations.INTAKE_ROLLERS_RUNNING_ANIMATION),
        //         blingSubsystem::clear,
        //         blingSubsystem))
        );
    dumpBalls.whileTrue(
        new IntakeRollerEject(
            intakeRollerSubsystem,
            () -> IntakeRollerConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE));

    // Launcher
    // Pose Based Triggers (Beta)
    //    FieldConstants.PoseZones.UnderTrenchZone.containsTrigger(swerveDriveSubsystem::getPose)
    //        .whileTrue(
    //            new DeflectorRunToPosition(
    //                deflectorSubsystem, DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE));
    //    // Alliance Zones
    //    FieldConstants.PoseZones.AllianceZoneBlue.containsTrigger(swerveDriveSubsystem::getPose)
    //        .and(RobotModeTriggers.teleop())
    //        .and(
    //            () ->
    //                DriverStation.getAlliance()
    //                    .orElse(DriverStation.Alliance.Blue)
    //                    .equals(DriverStation.Alliance.Blue))
    //        .whileTrue(
    //            new LauncherTargetHubDynamic(
    //                deflectorSubsystem, shooterSubsystem, turretSubsystem, launchCalculator));
    //    FieldConstants.PoseZones.AllianceZoneRed.containsTrigger(swerveDriveSubsystem::getPose)
    //        .and(RobotModeTriggers.teleop())
    //        .and(
    //            () ->
    //                DriverStation.getAlliance()
    //                    .orElse(DriverStation.Alliance.Blue)
    //                    .equals(DriverStation.Alliance.Red))
    //        .whileTrue(
    //            new LauncherTargetHubDynamic(
    //                deflectorSubsystem, shooterSubsystem, turretSubsystem, launchCalculator));
    //    // Passing Zones
    //    FieldConstants.PoseZones.PassingZoneBlue.containsTrigger(swerveDriveSubsystem::getPose)
    //        .and(RobotModeTriggers.teleop())
    //        .and(
    //            () ->
    //                DriverStation.getAlliance()
    //                    .orElse(DriverStation.Alliance.Blue)
    //                    .equals(DriverStation.Alliance.Blue))
    //        .whileTrue(
    //            new LauncherTargetPassingDynamic(
    //                deflectorSubsystem, shooterSubsystem, turretSubsystem, launchCalculator));
    //    FieldConstants.PoseZones.PassingZoneRed.containsTrigger(swerveDriveSubsystem::getPose)
    //        .and(RobotModeTriggers.teleop())
    //        .and(
    //            () ->
    //                DriverStation.getAlliance()
    //                    .orElse(DriverStation.Alliance.Blue)
    //                    .equals(DriverStation.Alliance.Red))
    //        .whileTrue(
    //            new LauncherTargetPassingDynamic(
    //                deflectorSubsystem, shooterSubsystem, turretSubsystem, launchCalculator));
    //
    //    turretAimShoot.whileTrue(
    //        new LauncherShoot(shooterSubsystem, beltSubsystem, rollerSubsystem, kickerSubsystem));

    // Buttons
    turretAimShoot.whileTrue(
        new LauncherTargetHubDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator));
    turretAimPass.whileTrue(
        new LauncherTargetPassingDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator));
    shoot.whileTrue(
        new LauncherTargetHubFixedAndShoot(
            beltSubsystem,
            kickerSubsystem,
            shooterSubsystem,
            deflectorSubsystem,
            turretSubsystem,
            LauncherTargetHubFixedAndShoot.FIXED_SHOOTING_POSITION_CENTER));

    // Climber
    toggleClimber.onTrue(
        new ConditionalCommand(
            new ClimberUpWithTurretSafety(climberSubsystem, turretSubsystem),
            new ClimberRunToExtension(
                climberSubsystem, ClimberConstants.Limits.MIN_CLIMB_EXTENSION),
            () -> {
              double currentPos = climberSubsystem.getClimberPosition().in(Meters);
              double minPos = ClimberConstants.Limits.MIN_CLIMB_EXTENSION.in(Meters);
              return Math.abs(currentPos - minPos) <= .05;
            }));

    // --- BACKUPS ---
    runKickerAndBeltManual.whileTrue(
        new DefaultKickerRunAtVelocity(
                kickerSubsystem, () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY)
            .alongWith(
                new DefaultBeltRunAtVelocity(
                    beltSubsystem, () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY)));
    runKickerAndBeltOutManual.whileTrue(
        new DefaultKickerRunAtVelocity(
                kickerSubsystem, () -> KickerConstants.Setpoints.LOAD_OUT_OF_SHOOTER_VELOCITY)
            .alongWith(
                new DefaultBeltRunAtVelocity(
                    beltSubsystem, () -> BeltConstants.Setpoints.LOAD_OUT_OF_SHOOTER_VELOCITY)));
    deflectorDownManual.onTrue(
        new InstantCommand(
            () ->
                deflectorSubsystem.runToTargetAngle(
                    deflectorSubsystem.getCurrentAngle().plus(Degrees.of(5)))));
    deflectorUpManual.onTrue(
        new InstantCommand(
            () ->
                deflectorSubsystem.runToTargetAngle(
                    deflectorSubsystem.getCurrentAngle().minus(Degrees.of(5)))));
    putDeflectorDown.onTrue(
        new DeflectorRunToPosition(
            deflectorSubsystem, DeflectorConstants.Limits.DEFLECTOR_MAX_ANGLE));
    // DATA COLLECTION ----------------------------
    OI.shootData.whileTrue(
        new LauncherTargetPassingDynamicAndShoot(
            deflectorSubsystem,
            shooterSubsystem,
            turretSubsystem,
            kickerSubsystem,
            beltSubsystem,
            rollerSubsystem,
            launchCalculator,
            () -> shooterVelocityOverride,
            () -> deflectorAngleOverride));
    OI.deflectorDownData.onTrue(
        new InstantCommand(
            () -> deflectorAngleOverride = deflectorAngleOverride.plus(Degrees.of(2.5))));
    OI.deflectorUpData.onTrue(
        new InstantCommand(
            () -> deflectorAngleOverride = deflectorAngleOverride.minus(Degrees.of(2.5))));
    OI.rpmDownData.onTrue(
        new InstantCommand(
            () ->
                shooterVelocityOverride =
                    shooterVelocityOverride.minus(RotationsPerSecond.of(2.5))));
    OI.rpmUpData.onTrue(
        new InstantCommand(
            () ->
                shooterVelocityOverride =
                    shooterVelocityOverride.plus(RotationsPerSecond.of(2.5))));
    OI.logData.onTrue(
        new InstantCommand(launchCalculator::clearPassingLaunchingParameters)
            .andThen(
                new InstantCommand(
                    () ->
                        Logger.recordOutput(
                            "DataCollection/data",
                            String.format(
                                "Turret Deg: %f, Deflector Deg: %f, RPS: %f, Distance: %f",
                                turretSubsystem.getCurrentAngle().in(Degrees),
                                deflectorAngleOverride.in(Degrees),
                                shooterVelocityOverride.in(RotationsPerSecond),
                                launchCalculator
                                    .getPassingTargetParameters()
                                    .dataCollectionDebugDistance()
                                    .in(Meters))))));
    // ------------------------------------------------
    // Sys id for turret

    sysIDDynamicFwd.whileTrue(
        shooterSubsystem.sysIdFwdDynamic().andThen(new InstantCommand(shooterSubsystem::stop)));
    sysIDDynamicRev.whileTrue(
        shooterSubsystem.sysIdRvsDynamic().andThen(new InstantCommand(shooterSubsystem::stop)));
    sysIDQuasistaticFwd.whileTrue(
        shooterSubsystem.sysIdFwdQuasistatic().andThen(new InstantCommand(shooterSubsystem::stop)));
    sysIDQuasistaticRev.whileTrue(
        shooterSubsystem.sysIdRvsQuasiStatic().andThen(new InstantCommand(shooterSubsystem::stop)));

    // Sys id for shooter
    /*
     * */
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
