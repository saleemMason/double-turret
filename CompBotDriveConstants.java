// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static final double JOYSTICK_TRANSLATION_FACTOR = 1.0;
  private static final double JOYSTICK_ROTATION_FACTOR = 0.75;

  private DriveCommands() {}

  private static ChassisSpeeds getLinearVelocityFromJoysticks(
      double x, double y, double speedMetersPerSec) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    Pose2d velocityAsPose =
        new Pose2d(
                Translation2d.kZero,
                AllianceUtil.shouldFlip()
                    ? linearDirection.plus(Rotation2d.k180deg)
                    : linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero));

    // Return new linear velocity
    return new ChassisSpeeds(
        velocityAsPose.getX() * speedMetersPerSec, velocityAsPose.getY() * speedMetersPerSec, 0.0);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          ChassisSpeeds speeds =
              getLinearVelocityFromJoysticks(
                  xSupplier.getAsDouble(),
                  ySupplier.getAsDouble(),
                  drive.getMaxLinearSpeedMetersPerSec() * JOYSTICK_TRANSLATION_FACTOR);

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Apply rotation speed
          speeds.omegaRadiansPerSecond +=
              omega * drive.getMaxAngularSpeedRadPerSec() * JOYSTICK_ROTATION_FACTOR;

          // Convert to robot relative speeds & send command
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    DriveController driveController = drive.getDriveController();

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              ChassisSpeeds speeds =
                  getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(),
                      ySupplier.getAsDouble(),
                      drive.getMaxLinearSpeedMetersPerSec() * JOYSTICK_TRANSLATION_FACTOR);

              // Calculate angular speed
              speeds.omegaRadiansPerSecond +=
                  driveController.getHeadingCorrection(drive.getRotation(), rotationSupplier.get());

              // Convert to robot relative speeds & send command
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> driveController.reset(drive.getPose()));
  }

  public static void setOdometryPose(Pose2d robotPose, Drive drive, Vision vision) {
    drive.setPose(robotPose);
    vision.resetCameraImus(robotPose.getRotation());
  }

  public static Command resetGyro(Drive drive, Vision vision) {
    return Commands.runOnce(
        () -> {
          Pose2d resetPose =
              new Pose2d(
                  drive.getPose().getTranslation(),
                  AllianceUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero);

          setOdometryPose(resetPose, drive, vision);
        });
  }
}
