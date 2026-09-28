package frc.robot.subsystems.shooters.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.pid.ClosedLoopConstants;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

public class FlywheelIOVortex implements FlywheelIO {
  public static record FlywheelConstantsVortex(
      int mainVortexDeviceID, int followerVortexDeviceID, ClosedLoopConstants pid) {}

  private final SparkFlex mainVortex;
  private final SparkFlex followerVortex;
  private final RelativeEncoder mainEncoder;
  private final SparkClosedLoopController pidCotroller;

  public FlywheelIOVortex(FlywheelConstantsVortex constants) {
    mainVortex = new SparkFlex(constants.mainVortexDeviceID, MotorType.kBrushless);
    followerVortex = new SparkFlex(constants.followerVortexDeviceID, MotorType.kBrushless);
    mainEncoder = mainVortex.getEncoder();
    pidCotroller = mainVortex.getClosedLoopController();

    SparkFlexConfig baseConfig = new SparkFlexConfig();
    baseConfig.smartCurrentLimit(120, 120);
    baseConfig.secondaryCurrentLimit(120);
    baseConfig.inverted(false);
    baseConfig.idleMode(IdleMode.kCoast).voltageCompensation(12.0);
    baseConfig.closedLoop.maxOutput(1);
    baseConfig.closedLoop.minOutput(0);

    SparkFlexConfig mainConfig = new SparkFlexConfig();
    mainConfig.apply(baseConfig);
    mainConfig.closedLoop.apply(constants.pid.getSparkConfig());

    tryUntilOk(
        5,
        () ->
            mainVortex.configure(
                mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.apply(baseConfig);
    followerConfig.follow(mainVortex, true);
    tryUntilOk(
        5,
        () ->
            followerVortex.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    AtomicBoolean mainOk = new AtomicBoolean(true);

    ifOk(
        mainVortex,
        mainEncoder::getPosition,
        (value) -> inputs.position = Rotations.of(value),
        () -> mainOk.set(false));
    ifOk(
        mainVortex,
        mainEncoder::getVelocity,
        (value) -> inputs.velocity = RPM.of(value),
        () -> mainOk.set(false));

    ifOk(
        mainVortex,
        new DoubleSupplier[] {mainVortex::getAppliedOutput, mainVortex::getBusVoltage},
        (values) -> inputs.mainAppliedVoltage = Volts.of(values[0] * values[1]),
        () -> mainOk.set(false));
    ifOk(
        mainVortex,
        mainVortex::getOutputCurrent,
        (value) -> inputs.mainSupplyCurrent = Amps.of(value),
        () -> mainOk.set(false));
    ifOk(
        mainVortex,
        mainVortex::getMotorTemperature,
        (value) -> inputs.mainTemperature = Celsius.of(value),
        () -> mainOk.set(false));

    inputs.mainConnected = mainOk.get();

    AtomicBoolean followerOk = new AtomicBoolean(true);

    ifOk(
        followerVortex,
        new DoubleSupplier[] {followerVortex::getAppliedOutput, followerVortex::getBusVoltage},
        (values) -> inputs.followerAppliedVoltage = Volts.of(values[0] * values[1]),
        () -> followerOk.set(false));
    ifOk(
        followerVortex,
        followerVortex::getOutputCurrent,
        (value) -> inputs.mainSupplyCurrent = Amps.of(value),
        () -> followerOk.set(false));
    ifOk(
        followerVortex,
        followerVortex::getMotorTemperature,
        (value) -> inputs.followerTemperature = Celsius.of(value),
        () -> followerOk.set(false));

    inputs.followerConnected = followerOk.get();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    mainVortex.setVoltage(voltage);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    pidCotroller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }
}
