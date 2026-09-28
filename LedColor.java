package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.pid.PidConstants;

public class PivotIOSim implements PivotIO {
  public record PivotConstantsSim(
      DCMotor motor,
      double motorReduction,
      Mass armMass,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      Angle startingAngle,
      PidConstants pid) {}

  private double appliedVolts = 0.0;
  private Angle goalPosition;

  private final SingleJointedArmSim sim;
  private final DCMotor motor;
  private final double motorReduction;
  private final PidConstants pid;

  public PivotIOSim(PivotConstantsSim constants) {
    this.motor = constants.motor;
    this.motorReduction = constants.motorReduction;
    this.pid = constants.pid;

    double armLengthMeters = constants.armLength.in(Meters);
    double armMassKg = constants.armMass.in(Kilograms);
    double armMoi = SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg);
    sim =
        new SingleJointedArmSim(
            motor,
            armMoi,
            motorReduction,
            armLengthMeters,
            constants.minAngle.in(Radians),
            constants.maxAngle.in(Radians),
            true,
            constants.startingAngle.in(Radians));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (goalPosition != null) {
      appliedVolts =
          (goalPosition.in(Radians) - sim.getAngleRads()) * pid.getKP()
              + (0.0 - sim.getVelocityRadPerSec()) * pid.getKD();
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.position = Radians.of(sim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  private void runVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    goalPosition = null;
  }

  @Override
  public void runNeutral() {
    runVoltage(0.0);
  }

  @Override
  public void runVoltage(Voltage voltage) {
    runVoltage(voltage.in(Volts));
  }

  @Override
  public void setPosition(Angle position) {
    goalPosition = position;
  }
}
