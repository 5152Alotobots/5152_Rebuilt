package frc.alotobots.rebuilt.subsystems.launcher.shooter.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.shooter.ShooterSubsystem;

import java.util.function.Supplier;

public class ShooterShootAtVelocity extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<AngularVelocity> targetVelocity;
    
    public ShooterShootAtVelocity(ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> targetVelocity) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetVelocity = targetVelocity;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.runToTargetVelocity(targetVelocity.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
