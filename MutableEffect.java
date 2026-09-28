package frc.robot.subsystems.shooters.hood;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanID;
import frc.robot.util.ServoManager.ContinuousServo;
import frc.robot.util.pid.PidConstants;

public class HoodIOServo implements HoodIO {
  public static record HoodConstantsServo(
      CanID cancoderCanId,
      boolean servoInverted,
      double sensorToMechanismReduction,
      MagnetSensorConfigs encoderConfigs,
      PidConstants pid) {}

  private final ContinuousServo servo;
  private final CANcoder cancoder;
  private final HoodConstantsServo constants;

  private final StatusSignal<Angle> rawPositionRot;
  private final StatusSignal<AngularVelocity> rawVelocityRotPerSec;

  private boolean runClosedLoop = false;
  private double servoOutput = 0.0;
  private double goalPositionRot = 0.0;

  public HoodIOServo(HoodConstantsServo constants, ContinuousServo servo) {
    this.servo = servo;
    this.constants = constants;

    servo.setEnabled(true);
    servo.setPowered(true);

    this.cancoder = constants.cancoderCanId.getCancoder();
    rawPositionRot = cancoder.getPosition();
    rawVelocityRotPerSec = cancoder.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, rawPositionRot, rawVelocityRotPerSec);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor = constants.encoderConfigs;

    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

    cancoder.optimizeBusUtilization();
  }

  private double getPositionRot() {
    return rawPositionRot.getValueAsDouble() / constants.sensorToMechanismReduction;
  }

  private void updateClosedLoop(boolean cancoderOk) {
    if (cancoderOk) {
      double error = goalPositionRot - getPositionRot();
      if (Math.abs(error) > Units.degreesToRotations(0.5)) {
        servoOutput = error * constants.pid.getKP();
      } else {
        servoOutput = 0.0;
      }
    } else {
      servoOutput = 0.0;
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusCode cancoderStatus = BaseStatusSignal.refreshAll(rawPositionRot, rawVelocityRotPerSec);

    if (runClosedLoop) {
      updateClosedLoop(cancoderStatus.isOK());
    }

    servo.setOutput(constants.servoInverted ? -servoOutput : servoOutput);

    inputs.motorConnected = cancoderStatus.isOK();
    inputs.position = rawPositionRot.getValue().div(constants.sensorToMechanismReduction);
    inputs.velocity = rawVelocityRotPerSec.getValue().div(constants.sensorToMechanismReduction);
    inputs.appliedVoltage = servo.getSupplyVoltage().times(MathUtil.clamp(servoOutput, -1.0, 1.0));
    inputs.supplyCurrent = servo.getOutputCurrent();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    runClosedLoop = false;
    servoOutput = voltage.in(Volts) / 12.0;
  }

  @Override
  public void runClosedLoop(Angle position) {
    runClosedLoop = true;
    goalPositionRot = position.in(Rotations);
  }
}
