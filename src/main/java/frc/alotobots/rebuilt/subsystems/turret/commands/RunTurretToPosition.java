package frc.alotobots.rebuilt.subsystems.turret.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.rebuilt.subsystems.turret.TurretSubsystem;

public class RunTurretToPosition extends Command {
    private final TurretSubsystem turretSubsystem;
    private final Angle targetAngle;

    public RunTurretToPosition(TurretSubsystem turretSubsystem, Angle targetAngle) {
        this.turretSubsystem = turretSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(turretSubsystem);
    }
    
    @Override
    public void execute() {
        turretSubsystem.runToTargetAngle(targetAngle);
    }
    
    @Override
    public void end(boolean interrupted) {

        turretSubsystem.stop();

        if (interrupted) {
            DataLogManager.log("INFO: Turret Position Command Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.isAtTargetAngle();
    }
}
