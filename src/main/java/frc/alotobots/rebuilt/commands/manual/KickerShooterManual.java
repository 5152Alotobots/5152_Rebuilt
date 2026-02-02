package frc.alotobots.rebuilt.commands.manual;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.rebuilt.subsystems.hopper.HopperSubsystem;
import frc.alotobots.rebuilt.subsystems.launcher.ShooterSubsystem;

public class KickerShooterManual extends SequentialCommandGroup {
    public KickerShooterManual(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, Supplier<AngularVelocity> velocity, Trigger launchButton) {
        addCommands(
            new InstantCommand(() -> shooterSubsystem.runToTargetVelocity(velocity.get())),
            new WaitUntilCommand(launchButton),
            new InstantCommand(() -> hopperSubsystem.runKickerToTargetVelocity(RotationsPerSecond.of(50)))
        );
    } 
}
