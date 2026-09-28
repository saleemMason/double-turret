// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.builtintest.BuiltInTest.Severity;
import frc.robot.util.builtintest.BuiltInTestSystem;
import frc.robot.util.builtintest.DisconnectedBIT;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase implements BuiltInTestSystem {
  private final VisionConstants constants;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final DisconnectedBIT[] disconnectedTests;

  private boolean disablePoseUpdates = false;

  public Vision(VisionConstants constants, VisionConsumer consumer, VisionIO... io) {
    this.constants = constants;
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedTests = new DisconnectedBIT[io.length];
    for (int i = 0; i < inputs.length; i++) {
      String cameraName = io[i].getCameraConstants().name();
      disconnectedTests[i] =
          new DisconnectedBIT(
              cameraName, Severity.WARNING, "Vision camera " + cameraName + " is disconnected.");
    }
  }

  @Override
  public String getTestSystemName() {
    return getName();
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + io[i].getCameraConstants().name(), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedTests[cameraIndex].setConnected(inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > constants.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > constants.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = constants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = constants.angularStdDevBaseline * stdDevFactor;

        if (observation.type() == PoseObservationType.MEGATAG_1) {
          linearStdDev *= constants.linearStdDevMegatag1Factor;
          angularStdDev *= constants.angularStdDevMegatag1Factor;
        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= constants.linearStdDevMegatag2Factor;
          angularStdDev *= constants.angularStdDevMegatag2Factor;
        }

        var cameraStdDevFactor = io[cameraIndex].getCameraConstants().stdDevFactor();

        linearStdDev *= cameraStdDevFactor;
        angularStdDev *= cameraStdDevFactor;

        // Send vision observation
        if (!disablePoseUpdates) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      String cameraLogTableName = "Vision/" + io[cameraIndex].getCameraConstants().name();

      // Log camera datadata
      Logger.recordOutput(
          cameraLogTableName + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          cameraLogTableName + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          cameraLogTableName + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          cameraLogTableName + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  public void resetCameraImus(Rotation2d robotOrientation) {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].resetCameraImu(robotOrientation);
    }
  }

  public void disablePoseUpdates(boolean disablePoseUpdates) {
    this.disablePoseUpdates = disablePoseUpdates;
  }

  public Command disablePoseUpdatedCmd() {
    return startEnd(() -> disablePoseUpdates(true), () -> disablePoseUpdates(false));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
