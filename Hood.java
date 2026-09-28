package frc.robot.robots.compbot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.robots.compbot.CompBotTunerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.pid.PidConstants;

public class CompBotDriveConstants
    extends DriveConstants<TalonFXConfiguration, CANcoderConfiguration> {
  @Override
  public CANBus getCanBus() {
    return CompBotTunerConstants.kCANBus;
  }

  @Override
  public LinearVelocity getSpeedAt12Volts() {
    return CompBotTunerConstants.kSpeedAt12Volts;
  }

  private static final Current driveCurrentLimit = Amps.of(60);
  private static final Current turnCurrentLimit = Amps.of(20);

  private static final ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> frontLeft =
      new ModuleConstants<>(CompBotTunerConstants.FrontLeft, driveCurrentLimit, turnCurrentLimit);
  private static final ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> frontRight =
      new ModuleConstants<>(CompBotTunerConstants.FrontRight, driveCurrentLimit, turnCurrentLimit);
  private static final ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> backLeft =
      new ModuleConstants<>(CompBotTunerConstants.BackLeft, driveCurrentLimit, turnCurrentLimit);
  private static final ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> backRight =
      new ModuleConstants<>(CompBotTunerConstants.BackRight, driveCurrentLimit, turnCurrentLimit);

  @Override
  public ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
    return frontLeft;
  }

  @Override
  public ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
    return frontRight;
  }

  @Override
  public ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
    return backLeft;
  }

  @Override
  public ModuleConstants<TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
    return backRight;
  }

  @Override
  public SwerveDrivetrainConstants getDrivetrain() {
    return CompBotTunerConstants.DrivetrainConstants;
  }

  private static final Mass kRobotMass = Pounds.of(115.2);
  private static final MomentOfInertia kRobotMoi = KilogramSquareMeters.of(6.883);
  private static final double kWheelCof = 1.2;

  @Override
  public Mass getRobotMass() {
    return kRobotMass;
  }

  @Override
  public MomentOfInertia getRobotMoi() {
    return kRobotMoi;
  }

  @Override
  public double getWheelCof() {
    return kWheelCof;
  }

  @Override
  public PidConstants getTranslationPid() {
    return new PidConstants().withKP(5.0);
  }

  @Override
  public PidConstants getRotationPid() {
    return new PidConstants().withKP(5.0);
  }
}
