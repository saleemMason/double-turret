package frc.robot.commands.aiming;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.aiming.AimCalculator.AimingParams;
import frc.robot.commands.aiming.AimCalculator.HoodInterference;
import frc.robot.commands.aiming.AimCalculator.ShooterAimingParams;
import frc.robot.commands.aiming.AimCalculator.TurretAimingParams;
import frc.robot.subsystems.shooters.ShooterSide;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AimController extends SubsystemBase {
  public final AimingStyle hub;
  public final AimingStyle passing;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotRelativeVelocitySupplier;

  private static final Angle turretOffsetIncrement = Degrees.of(2);
  private static final Angle hoodOffsetIncrement = Degrees.of(1);

  @AutoLogOutput private MutAngle turretOffset = Radians.mutable(0);
  @AutoLogOutput private MutAngle hoodOffset = Radians.mutable(0);
  @AutoLogOutput private MutAngularVelocity flywheelSpeedOffset = RPM.mutable(0);

  private Supplier<AimingParams> aimingParamsSupplier = null;
  private AimingParams currentAimingParams = null;

  private void setParamsSupplier(Supplier<AimingParams> supplier, String paramsSourceName) {
    this.aimingParamsSupplier = supplier;
    Logger.recordOutput(
        "AimController/AimingParamsSource", aimingParamsSupplier == null ? "" : paramsSourceName);
  }

  private void clearParamsSupplier() {
    setParamsSupplier(null, null);
  }

  public AimController(
      AimCalculator hubAimCalculator,
      AimCalculator passingAimCalculator,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotRelativeVelocitySupplier) {
    this.hub = new AimingStyle(hubAimCalculator);
    this.passing = new AimingStyle(passingAimCalculator);
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotRelativeVelocitySupplier = robotRelativeVelocitySupplier;
    clearParamsSupplier();
  }

  public Trigger hasValidTarget =
      new Trigger(
          () ->
              !(aimingParamsSupplier == null
                  || currentAimingParams == null
                  || currentAimingParams.shotObstructed()));

  public AimingParams getCurrentParams() {
    return currentAimingParams;
  }

  @Override
  public void periodic() {
    // Update aim in periodic because it runs before any scheduled commands
    Logger.recordOutput("AimController/HasParamsSupplier", aimingParamsSupplier != null);
    if (aimingParamsSupplier != null) {
      currentAimingParams = aimingParamsSupplier.get();
    } else {
      currentAimingParams = null;
    }

    Logger.recordOutput("AimController/HasValidTarget", currentAimingParams != null);
    if (currentAimingParams == null) {
      return;
    }

    Logger.recordOutput(
        "AimController/Turret/FieldRelativeAimRotation",
        currentAimingParams.turret().fieldRelativeAimRotation());
    Logger.recordOutput(
        "AimController/Turret/RobotRelativeTurretAngle",
        currentAimingParams.turret().robotRelativeTurretAngle());
    Logger.recordOutput(
        "AimController/Turret/TurretToTargetDistance",
        currentAimingParams.turret().turretToTargetDistance());

    Logger.recordOutput("AimController/HoodInterference", currentAimingParams.hoodInterference());

    Logger.recordOutput(
        "AimController/Left/HoodAngle", currentAimingParams.leftShooter().hoodAngle());
    Logger.recordOutput(
        "AimController/Left/FlywheelSpeed", currentAimingParams.leftShooter().flywheelSpeed());
    Logger.recordOutput(
        "AimController/Left/ShooterToTargetDistance",
        currentAimingParams.leftShooter().shooterToTargetDistance());

    Logger.recordOutput(
        "AimController/Right/HoodAngle", currentAimingParams.rightShooter().hoodAngle());
    Logger.recordOutput(
        "AimController/Right/FlywheelSpeed", currentAimingParams.rightShooter().flywheelSpeed());
    Logger.recordOutput(
        "AimController/Right/ShooterToTargetDistance",
        currentAimingParams.rightShooter().shooterToTargetDistance());
  }

  private Angle getTurretAngle() {
    if (currentAimingParams != null) {
      return currentAimingParams.turret().robotRelativeTurretAngle().plus(turretOffset);
    } else {
      return Radians.zero();
    }
  }

  /**
   * Make the turret aim using this AimController. The turret will use a limited range of motion to
   * track the desired aim
   *
   * @param turret The turret to control
   * @return A command to control the turret
   */
  public Command controlTurretTracking(Turret turret) {
    return turret.run(() -> turret.setTrackingRotation(getTurretAngle()));
  }

  /**
   * Make the turret aim using this AimController. The turret will use it's full range of motion to
   * fire at the desired aim
   *
   * @param turret The turret to control
   * @return A command to control the turret
   */
  public Command controlTurretFiring(Turret turret) {
    return turret.run(() -> turret.setFireRotation(getTurretAngle()));
  }

  private ShooterAimingParams getCurrentShooterParams(ShooterSide side) {
    return switch (side) {
      case LEFT -> currentAimingParams.leftShooter();
      case RIGHT -> currentAimingParams.rightShooter();
    };
  }

  private Angle getHoodAngle(ShooterSide side) {
    return getCurrentShooterParams(side).hoodAngle().plus(hoodOffset);
  }

  /**
   * Make the hood aim using this AimController
   *
   * @param hood The hood to control
   * @return A command to control the hood
   */
  public Command controlHood(Hood hood) {
    return hood.run(
            () -> {
              if (currentAimingParams != null) {
                hood.setHoodAngle(getHoodAngle(hood.getSide()));
              }
            })
        .withName("controlHood");
  }

  /**
   * Make the hood aim using this AimController while not going to a height unsafe for the trench
   *
   * @param hood The hood to control
   * @return A command to control the hood
   */
  public Command controlHoodSafe(Hood hood) {
    return hood.run(
            () -> {
              if (currentAimingParams != null) {
                hood.setHoodAngle(hood.getAngleOrSafeAngle(getHoodAngle(hood.getSide())));
              }
            })
        .withName("controlHoodSafe");
  }

  /**
   * Make the flywheel spin up using this AimController
   *
   * @param flywheel The flywheel to control
   * @return A command to control the flywheel
   */
  public Command controlFlywheel(Flywheel flywheel) {
    return flywheel.runEnd(
        () -> {
          if (currentAimingParams != null) {
            flywheel.runVelocity(
                getCurrentShooterParams(flywheel.getSide())
                    .flywheelSpeed()
                    .plus(flywheelSpeedOffset));
          } else {
            flywheel.runCoast();
          }
        },
        () -> flywheel.runCoast());
  }

  /*
   * Reset all manual shooting offsets
   */
  public Command resetOffsets() {
    return Commands.runOnce(
        () -> {
          turretOffset.mut_replace(Radians.zero());
          hoodOffset.mut_replace(Radians.zero());
          flywheelSpeedOffset.mut_replace(RPM.zero());
        });
  }

  public Command turretLeft() {
    return Commands.runOnce(() -> turretOffset.mut_plus(turretOffsetIncrement));
  }

  public Command turretRight() {
    return Commands.runOnce(() -> turretOffset.mut_minus(turretOffsetIncrement));
  }

  public Command hoodFurther() {
    return Commands.runOnce(() -> hoodOffset.mut_plus(hoodOffsetIncrement));
  }

  public Command hoodCloser() {
    return Commands.runOnce(() -> hoodOffset.mut_minus(hoodOffsetIncrement));
  }

  public Command aim(String paramsSourceName, Supplier<AimingParams> supplier) {
    return startEnd(
            () -> {
              setParamsSupplier(supplier, paramsSourceName);
            },
            this::clearParamsSupplier)
        .withName(paramsSourceName);
  }

  /** Aim a static shot */
  public Command aimStaticShot(Angle hoodAngle, AngularVelocity flywheelSpeed) {
    return aim(
            "aimStaticShot",
            () -> {
              ShooterAimingParams shooterParams =
                  new ShooterAimingParams(hoodAngle, flywheelSpeed, Meters.zero());
              return new AimingParams(
                  new TurretAimingParams(Rotation2d.kZero, Degrees.zero(), Meters.zero()),
                  HoodInterference.BOTH_LOWER,
                  shooterParams,
                  shooterParams);
            })
        .beforeStarting(resetOffsets());
  }

  public class AimingStyle {
    private final AimCalculator aimCalculator;

    AimingStyle(AimCalculator aimController) {
      this.aimCalculator = aimController;
    }

    public AimingParams getAimAtTargetParams(Translation2d target) {
      if (target == null) {
        return null;
      }

      return aimCalculator.calculateShot(
          target, robotPoseSupplier.get(), robotRelativeVelocitySupplier.get());
    }

    /**
     * Set the aiming subsystem to aim at the specified target
     *
     * @param target The field relative target point
     */
    public Command aimAtTarget(Translation2d target) {
      return aim("aimAtTarget", () -> getAimAtTargetParams(target));
    }

    /**
     * Set the aiming subsystem to aim at the specified target
     *
     * @param targetSupplier Supplier for the field relative target point. May supply null values.
     */
    public Command aimAtTarget(Supplier<Translation2d> targetSupplier) {
      return aim("aimAtTarget", () -> getAimAtTargetParams(targetSupplier.get()));
    }

    public AimingParams getAimAngleDistanceParams(
        Rotation2d fieldRelativeRotation, Distance distance) {
      return aimCalculator.calculateShot(fieldRelativeRotation, distance, robotPoseSupplier.get());
    }

    /**
     * Set the aiming subsystem to aim at the rotation and distance
     *
     * @param fieldRelativeRotation The field relative target angle
     * @param distance The distance to shoot
     */
    public Command aimAngleDistance(Rotation2d fieldRelativeRotation, Distance distance) {
      return aim(
          "aimAngleDistance", () -> getAimAngleDistanceParams(fieldRelativeRotation, distance));
    }
  }
}
