package frc.robot.subsystems.shooters.hood;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanID;
import frc.robot.util.pid.ClosedLoopConstants;

public class HoodIOTalonFX implements HoodIO {
  public record HoodConstantsTalonFX(
      CanID canId,
      InvertedValue hoodInvert,
      double motorReduction,
      Angle minAngle,
      Angle maxAngle,
      ClosedLoopConstants pid) {}

  private final TalonFX hoodMotor;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

  public HoodIOTalonFX(HoodConstantsTalonFX constants) {
    hoodMotor = constants.canId.getTalon();
    positionRot = hoodMotor.getPosition();
    velocityRotPerSec = hoodMotor.getVelocity();
    appliedVolts = hoodMotor.getMotorVoltage();
    currentAmps = hoodMotor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = constants.hoodInvert;

    config.Feedback.SensorToMechanismRatio = constants.motorReduction;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.minAngle.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.maxAngle.in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.Slot0 = constants.pid.getTalonSlot0();

    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.motorConnected = motorStatus.equals(StatusCode.OK);
    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.supplyCurrent = currentAmps.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    hoodMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void runClosedLoop(Angle hoodAngle) {
    hoodMotor.setControl(positionRequest.withPosition(hoodAngle));
  }
}
