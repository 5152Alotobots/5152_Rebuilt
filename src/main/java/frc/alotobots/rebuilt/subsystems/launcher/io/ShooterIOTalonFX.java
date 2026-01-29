package frc.alotobots.rebuilt.subsystems.launcher.io;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOTalonFX implements ShooterIO { 
    public ShooterIOTalonFX() {
        // Constructor implementation
    }

    @Override
    public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
        // Update inputs implementation
    }

    @Override
    public void setShooterVelocity(AngularVelocity velocity, PIDSlots pidSlot) {
        // Set shooter velocity with PID slot implementation
    }

    @Override
    public void setShooterVelocity(AngularVelocity velocity) {
        // Set shooter velocity implementation
        setShooterVelocity(velocity, PIDSlots.DEFAULT_VELOCITY);
    }

    @Override
    public void setShooterOpenLoop(double percentOutput) {
        // Set shooter open loop implementation
    }

    @Override
    public void stop() {
        // Stop shooter implementation
    }

}
