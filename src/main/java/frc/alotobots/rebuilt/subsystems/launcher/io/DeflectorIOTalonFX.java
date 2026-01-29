package frc.alotobots.rebuilt.subsystems.launcher.io;

import edu.wpi.first.units.measure.Angle;

public class DeflectorIOTalonFX implements DeflectorIO {
    public DeflectorIOTalonFX() {
        // Constructor implementation
    }

    @Override
    public void updateInputs(DeflectorIO.DeflectorIOInputs inputs) {
        // Update inputs implementation
    }

    @Override
    public void setDeflectorPosition(Angle position, PIDSlots pidSlot) {
        // Set deflector position with PID slot implementation
    }

    @Override
    public void setDeflectorOpenLoop(double percentOutput) {
        // Set deflector open loop implementation
    }

    @Override
    public void stop() {
        // Stop deflector implementation
    } 
}
