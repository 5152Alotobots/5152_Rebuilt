/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2026 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.rebuilt.subsystems.launcher.turret.io;

import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.alotobots.rebuilt.subsystems.launcher.turret.constants.TurretTalonFXSConstants;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class TurretIOSim extends TurretIOTalonFXS {
  private final DCMotorSim turretSim;
  private final TalonFXSSimState simState;

  private final LinearSystem<N2, N1, N2> sysIdPlant =
      LinearSystemId.createDCMotorSystem(
          TurretTalonFXSConstants.TURRET_KV, TurretTalonFXSConstants.TURRET_KA);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          DCMotor.getMinion(1),
          TurretTalonFXSConstants.MOMENT_OF_INERTIA,
          TurretTalonFXSConstants.TURRET_SENSOR_TO_MECHANISM_RATIO);

  public TurretIOSim() {
    super();

    turretSim = new DCMotorSim(plant, DCMotor.getMinion(1));
    simState = turretMotor.getSimState();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    turretSim.setInputVoltage(simState.getMotorVoltage());
    turretSim.update(0.020);

    simState.setRawRotorPosition(
        turretSim.getAngularPositionRotations()
            * TurretTalonFXSConstants.TURRET_SENSOR_TO_MECHANISM_RATIO);
    simState.setRotorVelocity(
        (turretSim.getAngularVelocityRPM() / 60.0)
            * TurretTalonFXSConstants.TURRET_SENSOR_TO_MECHANISM_RATIO);

    super.updateInputs(inputs);
  }
}
