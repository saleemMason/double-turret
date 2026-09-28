// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.drive.DriveConstants.DeviceConstants;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX(DeviceConstants deviceConstants) {
    navX = new AHRS(NavXComType.kMXP_SPI, (byte) deviceConstants.odometryFrequency());

    yawTimestampQueue = PhoenixOdometryThread.getInstance(deviceConstants).makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance(deviceConstants).registerSignal(navX::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.supplyVoltage = Volts.of(RobotController.getBatteryVoltage());
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
