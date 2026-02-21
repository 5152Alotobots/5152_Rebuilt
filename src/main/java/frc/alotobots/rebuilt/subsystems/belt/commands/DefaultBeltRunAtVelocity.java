package frc.alotobots.rebuilt.subsystems.belt.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.belt.BeltSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DefaultBeltRunAtVelocity extends Command {
    private final BeltSubsystem beltSubsystem;
    private final Supplier<AngularVelocity> velocitySupplier;
    public DefaultBeltRunAtVelocity(BeltSubsystem beltSubsystem, Supplier<AngularVelocity> velocitySupplier) {
        this.beltSubsystem = beltSubsystem;
        this.velocitySupplier = velocitySupplier;
        addRequirements(beltSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        beltSubsystem.runBeltToTargetVelocity(velocitySupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        beltSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
