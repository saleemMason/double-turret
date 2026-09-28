package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.pid.PidConstants;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim;
  private final DCMotor motor;
  private final double motorReduction;

  private final PidConstants pid;

  private double appliedVolts = 0.0;
  private boolean runClosedLoop = false;
  private Angle goalPosition;

  public TurretIOSim(DCMotor motor, double motorReduction, double turretMoi, PidConstants pid) {
    this.motor = motor;
    this.motorReduction = motorReduction;
    this.pid = pid;

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(this.motor, turretMoi, motorReduction), motor);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (runClosedLoop) {
      runVoltage(
          (goalPosition.in(Radians) - sim.getAngularPositionRad()) * pid.getKP()
              + (-sim.getAngularVelocityRadPerSec()) * pid.getKD());
    } else {
      appliedVolts = 0.0;
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.position = sim.getAngularPosition().div(motorReduction);
    inputs.velocity = sim.getAngularVelocity().div(motorReduction);
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  private void runVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTurretRotation(Angle position) {
    goalPosition = position;
    runClosedLoop = true;
  }

  @Override
  public void setTurretRotation(Angle position, AngularVelocity robotAngularVelocity) {
    setTurretRotation(position);
  }
}
