package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPositionAtVelocity;

public class LauncherTargetPassingDynamic extends SequentialCommandGroup {
    public LauncherTargetPassingDynamic(DeflectorSubsystem deflectorSubsystem,
                                        ShooterSubsystem shooterSubsystem,
                                        TurretSubsystem turretSubsystem,
                                        LaunchCalculator launchCalculator) {
        addCommands(
                new ParallelCommandGroup(
                        new DeflectorFollowPosition(
                                deflectorSubsystem,
                                () -> launchCalculator.getPassingTargetParameters().deflectorAngle()),
                        new DefaultShooterRunAtVelocity(
                                shooterSubsystem,
                                () -> launchCalculator.getPassingTargetParameters().shooterVelocity()),
                        new TurretFollowPositionAtVelocity(
                                turretSubsystem,
                                () -> launchCalculator
                                        .getPassingTargetParameters()
                                        .turretAngleFieldRelative()
                                        .getMeasure(),
                                () -> launchCalculator.getPassingTargetParameters().turretVelocity()),
                        new RunCommand(launchCalculator::clearPassingLaunchingParameters)));
    }
}
