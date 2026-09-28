package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

class AutoDriver {
  private final Drive drive;
  private final DriveController driveController;
  private final Vision vision;

  final AutoFactory autoFactory;

  public AutoDriver(Drive drive, Vision vision) {
    this.drive = drive;
    this.driveController = drive.getDriveController();
    this.vision = vision;

    autoFactory =
        new AutoFactory(
            drive::getPose,
            this::resetOdometry,
            this::driveChoreoTrajectory,
            true,
            drive,
            this::logTrajectory);
  }

  private void resetOdometry(Pose2d robotPose) {
    DriveCommands.setOdometryPose(robotPose, drive, vision);
  }

  private void driveChoreoTrajectory(SwerveSample sample) {
    Pose2d robotPose = drive.getPose();

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            sample.vx + driveController.getXCorrection(robotPose.getX(), sample.x),
            sample.vy + driveController.getYCorrection(robotPose.getY(), sample.y),
            sample.omega
                + driveController.getHeadingCorrection(
                    robotPose.getRotation().getRadians(), sample.heading));

    /*double[] forcesX = sample.moduleForcesX();
    double[] forcesY = sample.moduleForcesY();

    // The module forces are field relative, rotate them to be robot relative
    for (int i = 0; i < forcesX.length; i++) {
      Translation2d rotated =
          new Translation2d(forcesX[i], forcesY[i])
              .rotateBy(Rotation2d.fromRadians(-sample.heading));
      forcesX[i] = rotated.getX();
      forcesY[i] = rotated.getY();
    }*/

    // TODO: Use module forces
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotPose.getRotation()));
    logSetpointPose(sample.getPose());
  }

  private void logSetpointPose(Pose2d setpointPose) {
    Logger.recordOutput("Auto/SetpointPose", setpointPose);
  }

  private void logTrajectory(Trajectory<SwerveSample> trajectory, boolean starting) {
    if (starting) {
      Pose2d[] trajPoses = trajectory.getPoses();

      if (ChoreoAllianceFlipUtil.shouldFlip()) {
        for (int i = 0; i < trajPoses.length; i++) {
          trajPoses[i] = ChoreoAllianceFlipUtil.flip(trajPoses[i]);
        }
      }

      logTrajectory(trajPoses);
    } else {
      logTrajectory(new Pose2d[0]);
    }
  }

  private void logTrajectory(Pose2d[] trajPoses) {
    Logger.recordOutput("Auto/CurrentTrajectory", trajPoses);
  }
}
