package frc.robot.commands.aiming;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.commands.aiming.functions.ShotFunction;

/** Calculates different aiming strategies and returns the required data to aim the robot. */
public class AimCalculator {
  public record TurretLocationConstants(
      Transform2d robotToTurret,
      Translation2d turretToLeftShooter,
      Translation2d turretToRightShooter,
      Angle turretInterferenceAngle) {}

  public record AimingConstants(
      double robotPoseLookaheadSecs,
      int velocityCompIterations,
      ShotFunction lowerShot,
      ShotFunction upperShot) {}

  public enum HoodInterference {
    LEFT_OVER_RIGHT,
    RIGHT_OVER_LEFT,
    BOTH_LOWER;
  }

  public record TurretAimingParams(
      Rotation2d fieldRelativeAimRotation,
      Angle robotRelativeTurretAngle,
      Distance turretToTargetDistance) {}

  public record ShooterAimingParams(
      Angle hoodAngle, AngularVelocity flywheelSpeed, Distance shooterToTargetDistance) {}

  public record AimingParams(
      TurretAimingParams turret,
      HoodInterference hoodInterference,
      ShooterAimingParams leftShooter,
      ShooterAimingParams rightShooter,
      boolean shotObstructed) {
    public AimingParams(
        TurretAimingParams turret,
        HoodInterference hoodInterference,
        ShooterAimingParams leftShooter,
        ShooterAimingParams rightShooter) {
      this(turret, hoodInterference, leftShooter, rightShooter, false);
    }

    /**
     * Create a new AimingParams with the same aiming values and a different shotObstructed value
     *
     * @return The new AimingParams with shotObstructed set
     */
    public AimingParams withObstructed(boolean shotObstructed) {
      return new AimingParams(
          this.turret, this.hoodInterference, this.leftShooter, this.rightShooter, shotObstructed);
    }
  }

  private final TurretLocationConstants location;
  private final AimingConstants constants;
  private final double turretInterferenceSine;

  public AimCalculator(TurretLocationConstants location, AimingConstants constants) {
    this.location = location;
    this.constants = constants;
    turretInterferenceSine = Math.sin(location.turretInterferenceAngle.in(Radians));
  }

  private HoodInterference getHoodInterference(Rotation2d robotRelativeTurretRotation) {
    // 2d cross product (wedge product) with (1, 0) vector
    double cross = robotRelativeTurretRotation.getSin();
    if (Math.abs(cross) > turretInterferenceSine) {
      if (cross < 0) {
        return HoodInterference.LEFT_OVER_RIGHT;
      } else {
        return HoodInterference.RIGHT_OVER_LEFT;
      }
    } else {
      return HoodInterference.BOTH_LOWER;
    }
  }

  private AimingParams paramsFromAngleDistance(
      Rotation2d fieldRelativeTurretRotation,
      Rotation2d robotRotation,
      Distance turretToTargetDistance) {
    Rotation2d robotRelativeTurretRotation = fieldRelativeTurretRotation.minus(robotRotation);
    TurretAimingParams turretParams =
        new TurretAimingParams(
            fieldRelativeTurretRotation,
            robotRelativeTurretRotation.getMeasure(),
            turretToTargetDistance);

    Translation2d turretRelativeTarget =
        new Translation2d(
            turretToTargetDistance.in(Meters),
            new Rotation2d(turretParams.robotRelativeTurretAngle));
    Distance leftShooterDistance =
        Meters.of(turretRelativeTarget.getDistance(location.turretToLeftShooter));
    Distance rightShooterDistance =
        Meters.of(turretRelativeTarget.getDistance(location.turretToRightShooter));

    ShooterAimingParams leftParams;
    ShooterAimingParams rightParams;

    HoodInterference hoodInterference = getHoodInterference(robotRelativeTurretRotation);
    switch (hoodInterference) {
      case LEFT_OVER_RIGHT -> {
        leftParams = constants.upperShot.getParams(leftShooterDistance);
        rightParams = constants.lowerShot.getParams(rightShooterDistance);
      }

      case RIGHT_OVER_LEFT -> {
        leftParams = constants.lowerShot.getParams(leftShooterDistance);
        rightParams = constants.upperShot.getParams(rightShooterDistance);
      }

      default -> {
        leftParams = constants.lowerShot.getParams(leftShooterDistance);
        rightParams = constants.lowerShot.getParams(rightShooterDistance);
      }
    }

    return new AimingParams(turretParams, hoodInterference, leftParams, rightParams);
  }

  private double getBallTimeOfFlightSecs(double turretToTargetDistanceMeters) {
    Distance turretToTargetDistance = Meters.of(turretToTargetDistanceMeters);
    return (constants.lowerShot.getBallTimeOfFlightSecs(turretToTargetDistance)
            + constants.upperShot.getBallTimeOfFlightSecs(turretToTargetDistance))
        / 2;
  }

  public AimingParams calculateShot(
      Translation2d targetPoint, Pose2d robotPose, ChassisSpeeds robotRelativeVelocity) {
    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation());

    // Estimate the pose a short time in the future to more accurately aim the robot.
    Pose2d lookaheadRobotPose =
        robotPose.exp(robotRelativeVelocity.toTwist2d(constants.robotPoseLookaheadSecs));

    Pose2d turretPose = lookaheadRobotPose.transformBy(location.robotToTurret);
    double turretVelocityX =
        fieldRelativeVelocity.vxMetersPerSecond
            + fieldRelativeVelocity.omegaRadiansPerSecond
                * (location.robotToTurret.getY() * lookaheadRobotPose.getRotation().getCos()
                    - location.robotToTurret.getX() * lookaheadRobotPose.getRotation().getSin());
    double turretVelocityY =
        fieldRelativeVelocity.vyMetersPerSecond
            + fieldRelativeVelocity.omegaRadiansPerSecond
                * (location.robotToTurret.getX() * lookaheadRobotPose.getRotation().getCos()
                    - location.robotToTurret.getY() * lookaheadRobotPose.getRotation().getSin());

    double turretToTargetDistance = targetPoint.getDistance(turretPose.getTranslation());
    double ballTimeOfFlight;
    // target position accounting for the ball's imparted momentum from to the robot
    Translation2d momentumCompensatedTarget = targetPoint;
    for (int i = 0; i < constants.velocityCompIterations; i++) {
      ballTimeOfFlight = getBallTimeOfFlightSecs(turretToTargetDistance);
      // How far the ball will travel in relation to the target due to imparted momentum from the
      // robot
      double ballOffsetX = turretVelocityX * ballTimeOfFlight;
      double ballOffsetY = turretVelocityY * ballTimeOfFlight;

      momentumCompensatedTarget = targetPoint.minus(new Translation2d(ballOffsetX, ballOffsetY));
      turretToTargetDistance = turretPose.getTranslation().getDistance(momentumCompensatedTarget);
    }

    Rotation2d fieldRelativeTurretRotation =
        momentumCompensatedTarget.minus(turretPose.getTranslation()).getAngle();

    return paramsFromAngleDistance(
        fieldRelativeTurretRotation,
        lookaheadRobotPose.getRotation(),
        Meters.of(turretToTargetDistance));
  }

  public AimingParams calculateShot(
      Rotation2d fieldRelativeTurretRotation, Distance shotDistance, Pose2d robotPose) {
    return paramsFromAngleDistance(
        fieldRelativeTurretRotation, robotPose.getRotation(), shotDistance);
  }
}
