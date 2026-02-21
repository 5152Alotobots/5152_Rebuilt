package frc.alotobots.rebuilt.subsystems.kicker.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.kicker.KickerSubsystem;

import java.util.function.Supplier;

public class DefaultKickerRunAtVelocity extends Command {
    private final KickerSubsystem kickerSubsystem;
    private final Supplier<AngularVelocity> velocitySupplier;
    public DefaultKickerRunAtVelocity(KickerSubsystem kickerSubsystem, Supplier<AngularVelocity> velocitySupplier) {
        this.kickerSubsystem = kickerSubsystem;
        this.velocitySupplier = velocitySupplier;
        addRequirements(kickerSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        kickerSubsystem.runToTargetVelocity(velocitySupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        kickerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
