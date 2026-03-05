package frc.alotobots.library.subsystems.bling.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;

public class BlingHubShift extends Command {
    private final BlingSubsystem blingSubsystem;
    
    public BlingHubShift(BlingSubsystem blingSubsystem) {
            this.blingSubsystem = blingSubsystem;
            addRequirements(blingSubsystem);
    }
    
    @Override
    public void initialize() { }

    @Override
    public void execute() {
        blingSubsystem.setAnimation();
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
