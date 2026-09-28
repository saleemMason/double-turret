/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.pid.PidConstants;

public abstract class DriveConstants<
    MotorConfigs extends ParentConfiguration, EncoderConfigs extends ParentConfiguration> {
  public record DeviceConstants(CANBus canBus, double odometryFrequency) {}

  public record ModuleConstants<
      MotorConfigs extends ParentConfiguration, EncoderConfigs extends ParentConfiguration>(
      SwerveModuleConstants<MotorConfigs, MotorConfigs, EncoderConfigs> moduleConstants,
      Current driveCurrentLimit,
      Current turnCurrentLimit) {
    public Translation2d location() {
      return new Translation2d(moduleConstants.LocationX, moduleConstants.LocationY);
    }
  }

  public abstract LinearVelocity getSpeedAt12Volts();

  public abstract CANBus getCanBus();

  public abstract ModuleConstants<MotorConfigs, EncoderConfigs> getFrontRight();

  public abstract ModuleConstants<MotorConfigs, EncoderConfigs> getFrontLeft();

  public abstract ModuleConstants<MotorConfigs, EncoderConfigs> getBackLeft();

  public abstract ModuleConstants<MotorConfigs, EncoderConfigs> getBackRight();

  public abstract SwerveDrivetrainConstants getDrivetrain();

  public abstract Mass getRobotMass();

  public abstract MomentOfInertia getRobotMoi();

  public abstract double getWheelCof();

  public abstract PidConstants getTranslationPid();

  public abstract PidConstants getRotationPid();

  public double getOdometryFrequency() {
    return getCanBus().isNetworkFD() ? 250.0 : 100.0;
  }

  public DeviceConstants getDeviceConstants() {
    return new DeviceConstants(getCanBus(), getOdometryFrequency());
  }

  public double getDriveBaseRadius() {
    return Math.sqrt(
        Math.max(
            Math.max(
                getFrontLeft().location().getSquaredNorm(),
                getFrontRight().location().getSquaredNorm()),
            Math.max(
                getBackLeft().location().getSquaredNorm(),
                getBackRight().location().getSquaredNorm())));
  }
}
