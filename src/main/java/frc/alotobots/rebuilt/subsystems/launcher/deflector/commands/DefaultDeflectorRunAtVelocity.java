package frc.alotobots.rebuilt.subsystems.launcher.deflector.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.launcher.deflector.DeflectorSubsystem;

import java.util.function.DoubleSupplier;

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.rebuilt.subsystems.launcher.deflector.constants.DeflectorConstants.Limits.MAX_SPEED;

public class DefaultDeflectorRunAtVelocity extends Command {
    private final DeflectorSubsystem deflectorSubsystem;
    private final DoubleSupplier input;
    
    public DefaultDeflectorRunAtVelocity(DeflectorSubsystem deflectorSubsystem, DoubleSupplier input) {
        this.deflectorSubsystem = deflectorSubsystem;
        this.input = input;
        
        addRequirements(deflectorSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double adjustedInput = MathUtil.clamp(input.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT);
        AngularVelocity velocity = MAX_SPEED.times(adjustedInput);
        deflectorSubsystem.runToTargetVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
