package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;

public class AutoDriveCommands {
  private static final Distance TRANSLATION_MIN_ERROR = Inches.of(5);

  private static boolean atPoint(Drive drive, Translation2d targetPoint) {
    return drive.getPose().getTranslation().getDistance(targetPoint)
        < TRANSLATION_MIN_ERROR.in(Meters);
  }

  public static Command driveToPoint(Drive drive, Translation2d targetPoint) {
    DriveController driveController = drive.getDriveController();

    return drive
        .run(
            () -> {
              if (atPoint(drive, targetPoint)) {
                drive.runVelocity(new ChassisSpeeds());
              }

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveController.getTranslationCorrection(
                          drive.getPose().getTranslation(), targetPoint),
                      drive.getRotation()));
            })
        .until(() -> atPoint(drive, targetPoint))
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()))

        // Reset PID controller when command starts
        .beforeStarting(() -> driveController.reset(drive.getPose()));
  }

  public static Command driveToPose(Drive drive, Pose2d targetPose) {
    DriveController driveController = drive.getDriveController();

    return drive
        .run(
            () -> {
              if (atPoint(drive, targetPose.getTranslation())) {
                drive.runVelocity(new ChassisSpeeds());
              }

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveController.getPoseCorrection(drive.getPose(), targetPose),
                      drive.getRotation()));
            })
        .until(() -> atPoint(drive, targetPose.getTranslation()))
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()))

        // Reset PID controller when command starts
        .beforeStarting(() -> driveController.reset(drive.getPose()));
  }

  public static Command jitterAroundPose(
      Drive drive, Pose2d targetPose, Angle jitterAmount, Time jitterCycleTime) {
    double jitterOffsetRadians = jitterAmount.in(Radians) / 2;

    Pose2d minRotationPose =
        new Pose2d(
            targetPose.getTranslation(),
            targetPose.getRotation().minus(new Rotation2d(jitterOffsetRadians)));
    Pose2d maxRotationPose =
        new Pose2d(
            targetPose.getTranslation(),
            targetPose.getRotation().plus(new Rotation2d(jitterOffsetRadians)));

    DriveController driveController = drive.getDriveController();
    double cycleTimeSeconds = jitterCycleTime.in(Seconds);
    Timer jitterTimer = new Timer();

    return drive
        .run(
            () -> {
              jitterTimer.advanceIfElapsed(cycleTimeSeconds);
              Pose2d jitterTarget =
                  jitterTimer.hasElapsed(cycleTimeSeconds / 2) ? maxRotationPose : minRotationPose;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveController.getPoseCorrection(drive.getPose(), jitterTarget),
                      drive.getRotation()));
            })
        .finallyDo(() -> drive.runVelocity(new ChassisSpeeds()))

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              driveController.reset(drive.getPose());
              jitterTimer.restart();
            });
  }
}
