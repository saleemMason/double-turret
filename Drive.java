package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldLocations;
import frc.robot.util.AllianceUtil;
import java.util.function.Supplier;

public class RobotLocationTriggers {
  private static boolean isInBlueAllianceZone(Pose2d robotPose) {
    return robotPose.getX() < FieldLocations.blueHubCenterX.in(Meters);
  }

  public static Trigger inBlueAllianceZone(Supplier<Pose2d> robotPoseSupplier) {
    return new Trigger(() -> isInBlueAllianceZone(robotPoseSupplier.get()));
  }

  private static boolean isInRedAllianceZone(Pose2d robotPose) {
    return robotPose.getX() > FieldLocations.redHubCenterX.in(Meters);
  }

  public static Trigger inRedAllianceZone(Supplier<Pose2d> robotPoseSupplier) {
    return new Trigger(() -> isInRedAllianceZone(robotPoseSupplier.get()));
  }

  private static boolean isInNeutralZone(Pose2d robotPose) {
    return !(isInBlueAllianceZone(robotPose) || isInRedAllianceZone(robotPose));
  }

  public static Trigger inNeutralZone(Supplier<Pose2d> robotPoseSupplier) {
    return new Trigger(() -> isInNeutralZone(robotPoseSupplier.get()));
  }

  public static Trigger inOwnAllianceZone(Supplier<Pose2d> robotPoseSupplier) {
    return new Trigger(
        () ->
            AllianceUtil.shouldFlip()
                ? isInRedAllianceZone(robotPoseSupplier.get())
                : isInBlueAllianceZone(robotPoseSupplier.get()));
  }

  public static Trigger inOpposingAllianceZone(Supplier<Pose2d> robotPoseSupplier) {
    return new Trigger(
        () ->
            AllianceUtil.shouldFlip()
                ? isInBlueAllianceZone(robotPoseSupplier.get())
                : isInRedAllianceZone(robotPoseSupplier.get()));
  }
}
