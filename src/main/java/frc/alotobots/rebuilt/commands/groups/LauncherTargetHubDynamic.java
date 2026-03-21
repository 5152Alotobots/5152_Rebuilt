package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPositionAtVelocity;

public class LauncherTargetHubDynamic extends SequentialCommandGroup {
    public LauncherTargetHubDynamic(DeflectorSubsystem deflectorSubsystem,
                                    ShooterSubsystem shooterSubsystem,
                                    TurretSubsystem turretSubsystem,
                                    LaunchCalculator launchCalculator) {
        addCommands(
                new ParallelCommandGroup(
                        new DeflectorFollowPosition(
                                deflectorSubsystem,
                                () -> launchCalculator.getHubTargetParameters().deflectorAngle()),
                        new DefaultShooterRunAtVelocity(
                                shooterSubsystem,
                                () -> launchCalculator.getHubTargetParameters().shooterVelocity()),
                        new TurretFollowPositionAtVelocity(
                                turretSubsystem,
                                () -> launchCalculator
                                        .getHubTargetParameters()
                                        .turretAngleFieldRelative()
                                        .getMeasure(),
                                () -> launchCalculator.getHubTargetParameters().turretVelocity()),
                        new RunCommand(launchCalculator::clearHubLaunchingParameters)));
    }
}
