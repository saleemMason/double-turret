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

package frc.robot.subsystems.shooters.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanID;
import frc.robot.util.pid.ClosedLoopConstants;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class FlywheelIOTalonFX implements FlywheelIO {
  public record FlywheelConstantsTalonFX(
      CanID mainTalonCanId,
      CanID followerTalonCanId,
      InvertedValue flywheelInvert,
      double motorReduction,
      ClosedLoopConstants pid) {}

  protected final TalonFX mainMotor;
  protected final TalonFX followerMotor;

  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;

  private final StatusSignal<Voltage> mainAppliedVolts;
  private final StatusSignal<Current> mainCurrentAmps;
  private final StatusSignal<Temperature> mainTemperature;

  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrentAmps;
  private final StatusSignal<Temperature> followerTemperature;

  private final VoltageOut voltageRequest = new VoltageOut(Volts.zero());
  private final VelocityVoltage velocityRequest = new VelocityVoltage(RadiansPerSecond.zero());

  public FlywheelIOTalonFX(FlywheelConstantsTalonFX constants) {
    mainMotor = constants.mainTalonCanId.getTalon();
    followerMotor = constants.followerTalonCanId.getTalon();

    positionRot = mainMotor.getPosition();
    velocityRotPerSec = mainMotor.getVelocity();
    mainAppliedVolts = mainMotor.getMotorVoltage();
    mainCurrentAmps = mainMotor.getSupplyCurrent();
    mainTemperature = mainMotor.getDeviceTemp();

    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrentAmps = followerMotor.getSupplyCurrent();
    followerTemperature = followerMotor.getDeviceTemp();

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = constants.flywheelInvert;
    config.Feedback.SensorToMechanismRatio = constants.motorReduction;

    config.Slot0 = constants.pid.getTalonSlot0();

    tryUntilOk(5, () -> mainMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(
        5,
        () ->
            followerMotor.setControl(
                new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed)));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, mainAppliedVolts, mainCurrentAmps);
    mainMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    StatusCode mainStatus =
        BaseStatusSignal.refreshAll(
            positionRot, velocityRotPerSec, mainAppliedVolts, mainCurrentAmps, mainTemperature);

    inputs.mainConnected = mainStatus.equals(StatusCode.OK);
    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.mainAppliedVoltage = mainAppliedVolts.getValue();
    inputs.mainSupplyCurrent = mainCurrentAmps.getValue();
    inputs.mainTemperature = mainTemperature.getValue();

    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(followerAppliedVolts, followerCurrentAmps, followerTemperature);

    inputs.followerConnected = followerStatus.equals(StatusCode.OK);
    inputs.followerAppliedVoltage = followerAppliedVolts.getValue();
    inputs.followerSupplyCurrent = followerCurrentAmps.getValue();
    inputs.followerTemperature = followerTemperature.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    mainMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    mainMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
