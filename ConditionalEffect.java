package frc.robot.subsystems.shooters.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.pid.PidConstants;

public class FlywheelIOSim implements FlywheelIO {
  private double appliedVolts = 0.0;

  private final DCMotorSim sim;
  private final DCMotor motor;
  private final double motorReduction;

  private final PidConstants pid;

  public FlywheelIOSim(DCMotor motor, double motorReduction, double flywheelMoi, PidConstants pid) {
    this.motor = motor;
    this.motorReduction = motorReduction;
    this.pid = pid;

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(this.motor, flywheelMoi, motorReduction), motor);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.mainConnected = true;
    inputs.followerConnected = true;
    inputs.position = sim.getAngularPosition().div(motorReduction);
    inputs.velocity = sim.getAngularVelocity().div(motorReduction);
    inputs.mainAppliedVoltage = Volts.of(appliedVolts);
    inputs.followerAppliedVoltage = inputs.mainAppliedVoltage;
    inputs.mainSupplyCurrent = Amps.of(sim.getCurrentDrawAmps());
    inputs.followerSupplyCurrent = inputs.mainSupplyCurrent;
  }

  private void runVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    double velocityRadPerSec = velocity.in(RadiansPerSecond);

    runVoltage(
        1.0 / motor.KvRadPerSecPerVolt * velocityRadPerSec
            + (velocityRadPerSec - sim.getAngularVelocityRadPerSec()) * pid.getKP());
  }
}
