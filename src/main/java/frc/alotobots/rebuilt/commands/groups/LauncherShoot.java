package frc.alotobots.rebuilt.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;
import frc.alotobots.rebuilt.subsystems.belt.commands.DefaultBeltRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.belt.constants.BeltConstants;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;
import frc.alotobots.rebuilt.subsystems.kicker.commands.DefaultKickerRunAtVelocity;
import frc.alotobots.rebuilt.subsystems.kicker.constants.KickerConstants;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;

public class LauncherShoot extends SequentialCommandGroup {
    public LauncherShoot(ShooterSubsystem shooterSubsystem,
                         BeltSubsystem beltSubsystem,
                         KickerSubsystem kickerSubsystem) {
        addCommands(
                        new WaitUntilCommand(shooterSubsystem::isAtTargetVelocity),
                        new ParallelCommandGroup(
                                new DefaultBeltRunAtVelocity(
                                        beltSubsystem,
                                        () -> BeltConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY),
                                new DefaultKickerRunAtVelocity(
                                        kickerSubsystem,
                                        () -> KickerConstants.Setpoints.LOAD_INTO_SHOOTER_VELOCITY)));
    }
    }
