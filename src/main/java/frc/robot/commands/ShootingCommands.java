package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.FieldLocations;
import frc.robot.Robot.Subsystems;
import frc.robot.commands.aiming.AimController;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceUtil;
import java.util.function.BooleanSupplier;

public class ShootingCommands {
  /** Longest wait for the hoods and flywheels to be ready before feeding starts anyway. */
  private static final double readyTimeoutSecs = 1.0;

  public static Translation2d getAllianceHubLocation() {
    return AllianceUtil.shouldFlip() ? FieldLocations.redHubCenter : FieldLocations.blueHubCenter;
  }

  public static Command aimHoods(AimController aimController, Hood leftHood, Hood rightHood) {
    return Commands.parallel(
        aimController.controlHood(leftHood), aimController.controlHood(rightHood));
  }

  /**
   * Control the hoods ensuring they say below the safe angle. This command will not cancel iself
   * when a conflicting command is scheduled.
   */
  public static Command aimHoodsSafe(AimController aimController, Hood leftHood, Hood rightHood) {
    return Commands.parallel(
            aimController.controlHoodSafe(leftHood), aimController.controlHoodSafe(rightHood))
        .withName("aimHoodsSafe")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command spinFlywheels(
      AimController aimController, Flywheel leftFlywheel, Flywheel rightFlywheel) {
    return Commands.parallel(
            aimController.controlFlywheel(leftFlywheel),
            aimController.controlFlywheel(rightFlywheel))
        .withName("spinFlywheels");
  }

  public static Command prepShot(AimController aimController, Subsystems s) {
    return prepShot(
        aimController, s.leftHood(), s.rightHood(), s.leftFlywheel(), s.rightFlywheel());
  }

  public static Command prepShot(
      AimController aimController,
      Hood leftHood,
      Hood rightHood,
      Flywheel leftFlywheel,
      Flywheel rightFlywheel) {
    return Commands.parallel(
        aimHoods(aimController, leftHood, rightHood),
        spinFlywheels(aimController, leftFlywheel, rightFlywheel));
  }

  public static Command shoot(AimController aimController, Subsystems s) {
    return shoot(
        aimController,
        s.intake(),
        s.indexer(),
        s.turret(),
        s.leftHood(),
        s.rightHood(),
        s.leftFlywheel(),
        s.rightFlywheel());
  }

  public static Command shoot(
      AimController aimController,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Hood leftHood,
      Hood rightHood,
      Flywheel leftFlywheel,
      Flywheel rightFlywheel) {
    BooleanSupplier turretOkay = turret.atGoal.and(aimController.hasValidTarget);
    BooleanSupplier hoodsAtGoal = leftHood.readyToShoot.and(rightHood.readyToShoot);
    BooleanSupplier flywheelsAtGoal = leftFlywheel.atVelocity.and(rightFlywheel.atVelocity);

    return Commands.parallel(
        Commands.sequence(
            // Start feeding as soon as the hoods and flywheels are ready. The timeout only matters
            // when they never settle (for example while the target keeps moving).
            Commands.race(
                Commands.waitUntil(
                    () -> hoodsAtGoal.getAsBoolean() && flywheelsAtGoal.getAsBoolean()),
                Commands.waitSeconds(readyTimeoutSecs)),
            indexer.indexIntoShooter().onlyIf(turretOkay).onlyWhile(turretOkay).repeatedly()),
        intake
            .intake()
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .asProxy()
            .repeatedly(),
        aimController.controlTurretFiring(turret),
        aimHoods(aimController, leftHood, rightHood)
            .asProxy()
            .repeatedly()
            .until(RobotModeTriggers.disabled()),
        spinFlywheels(aimController, leftFlywheel, rightFlywheel)
            .asProxy()
            .repeatedly()
            .until(RobotModeTriggers.disabled()));
  }

  public static Command aimAtHub(AimController aimController) {
    return aimController.hub.aimAtTarget(ShootingCommands::getAllianceHubLocation);
  }

  public static Command zeroHoods(Hood leftHood, Hood rightHood) {
    return Commands.parallel(leftHood.zeroHood(), rightHood.zeroHood());
  }
}
