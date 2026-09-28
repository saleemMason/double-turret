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

package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanID;
import frc.robot.util.pid.ClosedLoopConstants;
import frc.robot.util.pid.MotionProfileConstants;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class TurretIOTalonFX implements TurretIO {
  public record TurretConstantsTalonFX(
      CanID canId,
      double motorReduction,
      InvertedValue motorInvert,
      Angle minAngle,
      Angle maxAngle,
      ClosedLoopConstants pid,
      MotionProfileConstants motionProfile) {}

  protected final TalonFX motor;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0.0);

  public TurretIOTalonFX(TurretConstantsTalonFX constants) {
    motor = constants.canId.getTalon();
    positionRot = motor.getPosition();
    velocityRotPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = constants.motorInvert;

    config.Feedback.SensorToMechanismRatio = constants.motorReduction;

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(constants.maxAngle);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(constants.minAngle);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0 = constants.pid.getTalonSlot0();
    config.MotionMagic = constants.motionProfile.getMotionMagic();

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.motorConnected = motorStatus.equals(StatusCode.OK);
    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.supplyCurrent = currentAmps.getValue();
  }

  private void setTurretRotation(Angle position, double feedforwardCurrentAmps) {
    motor.setControl(
        positionRequest.withPosition(position).withFeedForward(feedforwardCurrentAmps));
  }

  @Override
  public void setTurretRotation(Angle position) {
    setTurretRotation(position, 0);
  }

  @Override
  public void setTurretRotation(Angle position, AngularVelocity robotAngularVelocity) {
    setTurretRotation(position, 0);
  }
}
