// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.DeviceConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Voltage> supplyVoltage;

  public GyroIOPigeon2(SwerveDrivetrainConstants constants, DeviceConstants deviceConstants) {
    pigeon = new Pigeon2(constants.Pigeon2Id, deviceConstants.canBus());

    if (constants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(constants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);

    yawTimestampQueue = PhoenixOdometryThread.getInstance(deviceConstants).makeTimestampQueue();

    yaw = pigeon.getYaw();
    yaw.setUpdateFrequency(deviceConstants.odometryFrequency());
    yawPositionQueue =
        PhoenixOdometryThread.getInstance(deviceConstants).registerSignal(yaw.clone());

    yawVelocity = pigeon.getAngularVelocityZWorld();
    yawVelocity.setUpdateFrequency(50.0);

    supplyVoltage = pigeon.getSupplyVoltage();
    supplyVoltage.setUpdateFrequency(5.0);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, supplyVoltage).equals(StatusCode.OK);
    inputs.supplyVoltage = supplyVoltage.getValue();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
