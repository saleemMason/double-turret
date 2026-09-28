package frc.robot.commands.aiming;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldLocations;
import frc.robot.commands.aiming.AimCalculator.AimingParams;
import frc.robot.util.AllianceUtil;
import frc.robot.util.math.GeometryUtil.ClosestPoint;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PassingAimCommands {
  private static final Distance NET_HALF_HEIGHT_Y = Inches.of(58.41 / 2.0);
  private static final Distance NET_WIDTH_X = Inches.of(10.26).plus(Inches.of(47.0 / 2.0));
  private static final Distance NET_CENTER_Y = FieldLocations.fieldCenterY;
  private static final Distance BLUE_NET_REAR_X = FieldLocations.blueHubCenterX.plus(NET_WIDTH_X);
  private static final Distance RED_NET_REAR_X = FieldLocations.redHubCenterX.minus(NET_WIDTH_X);

  private static final Translation2d BLUE_NET_FL =
      new Translation2d(FieldLocations.blueHubCenterX, NET_CENTER_Y.plus(NET_HALF_HEIGHT_Y));
  private static final Translation2d BLUE_NET_BL =
      new Translation2d(BLUE_NET_REAR_X, NET_CENTER_Y.plus(NET_HALF_HEIGHT_Y));
  private static final Translation2d BLUE_NET_BR =
      new Translation2d(BLUE_NET_REAR_X, NET_CENTER_Y.minus(NET_HALF_HEIGHT_Y));
  private static final Translation2d BLUE_NET_FR =
      new Translation2d(FieldLocations.blueHubCenterX, NET_CENTER_Y.minus(NET_HALF_HEIGHT_Y));

  private static final Translation2d RED_NET_FL =
      new Translation2d(FieldLocations.redHubCenterX, NET_CENTER_Y.plus(NET_HALF_HEIGHT_Y));
  private static final Translation2d RED_NET_BL =
      new Translation2d(RED_NET_REAR_X, NET_CENTER_Y.plus(NET_HALF_HEIGHT_Y));
  private static final Translation2d RED_NET_BR =
      new Translation2d(RED_NET_REAR_X, NET_CENTER_Y.minus(NET_HALF_HEIGHT_Y));
  private static final Translation2d RED_NET_FR =
      new Translation2d(FieldLocations.redHubCenterX, NET_CENTER_Y.minus(NET_HALF_HEIGHT_Y));

  private static final Distance NET_MIN_CLEARANCE = Inches.of(10);

  private static final Translation2d[] passingSpots =
      new Translation2d[] {
        new Translation2d(Meters.of(1), Meters.of(-2.5).plus(NET_CENTER_Y)),
        new Translation2d(Meters.of(1), Meters.of(2.5).plus(NET_CENTER_Y))
      };

  private static double getShotDistanceToNet(Translation2d robotPose, Translation2d shotPose) {
    ClosestPoint p1 = ClosestPoint.segmentToSegment(shotPose, robotPose, BLUE_NET_FL, BLUE_NET_BL);
    ClosestPoint p2 = ClosestPoint.segmentToSegment(shotPose, robotPose, BLUE_NET_BL, BLUE_NET_BR);
    ClosestPoint p3 = ClosestPoint.segmentToSegment(shotPose, robotPose, BLUE_NET_BR, BLUE_NET_FR);

    ClosestPoint p4 = ClosestPoint.segmentToSegment(shotPose, robotPose, RED_NET_FL, RED_NET_BL);
    ClosestPoint p5 = ClosestPoint.segmentToSegment(shotPose, robotPose, RED_NET_BL, RED_NET_BR);
    ClosestPoint p6 = ClosestPoint.segmentToSegment(shotPose, robotPose, RED_NET_BR, RED_NET_FR);

    ClosestPoint closestPoint =
        ClosestPoint.min(
            ClosestPoint.min(ClosestPoint.min(p1, p2), ClosestPoint.min(p3, p4)),
            ClosestPoint.min(p5, p6));

    return closestPoint.distance();
  }

  private static AimingParams getParamsForPassingSpot(
      AimController aimController, Translation2d passingSpot) {
    return aimController.passing.getAimAtTargetParams(passingSpot);
  }

  /**
   * Returns aimingParams to use for passing
   *
   * @param robotPose
   * @return Passing aiming params or null if no passing spots exist
   */
  public static AimingParams getPassingAimingParams(AimController aimController, Pose2d robotPose) {
    if (passingSpots.length == 0) {
      return null;
    }

    Translation2d[] sortedPassingSpots = Arrays.copyOf(passingSpots, passingSpots.length);
    Arrays.sort(
        sortedPassingSpots,
        (a, b) ->
            // Sort by lowest distance
            Double.compare(
                b.getSquaredDistance(robotPose.getTranslation()),
                a.getSquaredDistance(robotPose.getTranslation())));

    for (Translation2d passingSpot : sortedPassingSpots) {
      Translation2d location = AllianceUtil.flip(passingSpot);
      double shotClearance = getShotDistanceToNet(robotPose.getTranslation(), location);

      if (shotClearance < NET_MIN_CLEARANCE.in(Meters)) {
        continue;
      }

      Logger.recordOutput("PassingAim/AimPosition", location);
      return getParamsForPassingSpot(aimController, location);
    }

    Translation2d location = AllianceUtil.flip(sortedPassingSpots[0]);
    Logger.recordOutput("PassingAim/AimPosition", location);
    return getParamsForPassingSpot(aimController, location).withObstructed(true);
  }

  public static Command aimAtPassingSpot(
      AimController aimController, Supplier<Pose2d> robotPoseSupplier) {
    return aimController.aim(
        "aimAtPassingSpot", () -> getPassingAimingParams(aimController, robotPoseSupplier.get()));
  }
}
