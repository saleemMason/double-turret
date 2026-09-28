package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.pid.PidConstants;

public class DriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController headingController;

  DriveController(PidConstants translationPid, PidConstants rotationPid) {
    xController = translationPid.getWpilibPidController();
    yController = translationPid.getWpilibPidController();
    headingController = rotationPid.getWpilibPidController();
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getXCorrection(double currentX, double targetX) {
    return xController.calculate(currentX, targetX);
  }

  public double getYCorrection(double currentY, double targetY) {
    return yController.calculate(currentY, targetY);
  }

  public double getHeadingCorrection(double currentHeading, double targetHeading) {
    return headingController.calculate(currentHeading, targetHeading);
  }

  public double getHeadingCorrection(Rotation2d currentHeading, Rotation2d targetHeading) {
    return headingController.calculate(currentHeading.getRadians(), targetHeading.getRadians());
  }

  public ChassisSpeeds getTranslationCorrection(Translation2d robotPose, Translation2d targetPose) {
    return new ChassisSpeeds(
        getXCorrection(robotPose.getX(), targetPose.getX()),
        getYCorrection(robotPose.getY(), targetPose.getY()),
        0.0);
  }

  public ChassisSpeeds getPoseCorrection(Pose2d robotPose, Pose2d targetPose) {
    return new ChassisSpeeds(
        getXCorrection(robotPose.getX(), targetPose.getX()),
        getYCorrection(robotPose.getY(), targetPose.getY()),
        getHeadingCorrection(robotPose.getRotation(), targetPose.getRotation()));
  }

  public void reset(Pose2d robotPose) {
    xController.reset();
    yController.reset();
    headingController.reset();
  }
}
