// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private double appliedVolts = 0.0;

  private final DCMotorSim sim;
  private final DCMotor motor;
  private final double motorReduction;

  public RollerIOSim(DCMotor motor, double motorReduction, double rollerMoi) {
    this.motor = motor;
    this.motorReduction = motorReduction;

    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, rollerMoi, motorReduction), motor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.position = sim.getAngularPosition().div(motorReduction);
    inputs.velocity = sim.getAngularVelocity().div(motorReduction);
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  private void runVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void runVoltage(Voltage voltage) {
    runVoltage(voltage.in(Volts));
  }

  @Override
  public void runTorque(Current current) {
    runVoltage(
        motor.getVoltage(motor.getTorque(current.in(Amps)), sim.getAngularVelocityRadPerSec()));
  }
}
