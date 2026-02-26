package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.alotobots.rebuilt.subsystems.launcher.LaunchCalculator;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.commands.DeflectorFollowPosition;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.commands.DefaultShooterRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.launcher.turret.TurretSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.turret.commands.TurretFollowPosition;

public class LauncherTargetHub extends ParallelCommandGroup {
    public LauncherTargetHub(DeflectorSubsystem deflectorSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, LaunchCalculator launchCalculator) {
        addCommands(
                new DeflectorFollowPosition(deflectorSubsystem, () -> launchCalculator.getParameters().deflectorAngle()),
                new DefaultShooterRunAtVelocity(shooterSubsystem, () -> launchCalculator.getParameters().shooterVelocity()),
                new TurretFollowPosition(turretSubsystem, () -> launchCalculator.getParameters().turretAngle().getMeasure()),
                new RunCommand(launchCalculator::clearLaunchingParameters)
        );
    }
}
