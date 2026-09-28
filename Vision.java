// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.util.builtintest.BuiltInTest.Severity;
import frc.robot.util.builtintest.BuiltInTestSystem;
import frc.robot.util.builtintest.DisconnectedBIT;
import org.littletonrobotics.junction.Logger;

public class Module implements BuiltInTestSystem {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          ? extends ParentConfiguration,
          ? extends ParentConfiguration,
          ? extends ParentConfiguration>
      moduleConstants;

  private final DisconnectedBIT driveDisconnectedTest;
  private final DisconnectedBIT turnDisconnectedTest;
  private final DisconnectedBIT turnEncoderDisconnectedTest;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      ModuleConstants<? extends ParentConfiguration, ? extends ParentConfiguration> constants) {
    this.io = io;
    this.index = index;
    this.moduleConstants = constants.moduleConstants();
    driveDisconnectedTest =
        new DisconnectedBIT(
            "drive",
            Severity.STARTUP_ERROR,
            "Disconnected drive motor on module " + Integer.toString(index) + ".");
    turnDisconnectedTest =
        new DisconnectedBIT(
            "turn",
            Severity.STARTUP_ERROR,
            "Disconnected turn motor on module " + Integer.toString(index) + ".");
    turnEncoderDisconnectedTest =
        new DisconnectedBIT(
            "encoder",
            Severity.STARTUP_ERROR,
            "Disconnected turn encoder on module " + Integer.toString(index) + ".");
  }

  @Override
  public String getTestSystemName() {
    return "Module" + index;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * moduleConstants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedTest.setConnected(inputs.driveConnected);
    turnDisconnectedTest.setConnected(inputs.turnConnected);
    turnEncoderDisconnectedTest.setConnected(inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / moduleConstants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * moduleConstants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * moduleConstants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
