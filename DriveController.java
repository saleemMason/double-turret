package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.aiming.AimCalculator.AimingParams;
import frc.robot.commands.aiming.AimController;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooters.ShooterSide;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShootingTuner {
  private LoggedNetworkNumber leftRpmInput =
      new LoggedNetworkNumber("/Tuning/Shooters/leftFlywheelRPM", 0.0);
  private LoggedNetworkNumber rightRpmInput =
      new LoggedNetworkNumber("/Tuning/Shooters/rightFlywheelRPM", 0.0);
  private LoggedNetworkNumber leftAngleInput =
      new LoggedNetworkNumber("/Tuning/Shooters/leftHoodDegrees", 0.0);
  private LoggedNetworkNumber rightAngleInput =
      new LoggedNetworkNumber("/Tuning/Shooters/rightHoodDegrees", 0.0);

  private final Alert tuningModeAlert =
      new Alert("Robot in tuning mode! Reboot code to disable.", AlertType.kInfo);

  private String allGeneratedCode = "";

  private Angle getHoodAngle(LoggedNetworkNumber tuningAngle, Angle paramsAngle) {
    double hoodDegrees = tuningAngle.get();
    if (hoodDegrees < 15) {
      return paramsAngle;
    } else {
      return Degrees.of(hoodDegrees);
    }
  }

  private AngularVelocity getFlywheelSpeed(
      LoggedNetworkNumber tuningSpeed, AngularVelocity paramsSpeed) {

    double flywheelRpm = tuningSpeed.get();
    if (flywheelRpm < 50) {
      return paramsSpeed;
    } else {
      return RPM.of(flywheelRpm);
    }
  }

  private Angle leftAngle(AimingParams params) {
    return getHoodAngle(leftAngleInput, params == null ? null : params.leftShooter().hoodAngle());
  }

  private AngularVelocity leftRpm(AimingParams params) {
    return getFlywheelSpeed(
        leftRpmInput, params == null ? null : params.leftShooter().flywheelSpeed());
  }

  private Angle rightAngle(AimingParams params) {
    return getHoodAngle(rightAngleInput, params == null ? null : params.rightShooter().hoodAngle());
  }

  private AngularVelocity rightRpm(AimingParams params) {
    return getFlywheelSpeed(
        rightRpmInput, params == null ? null : params.rightShooter().flywheelSpeed());
  }

  private static String generateLookupTableCode(
      ShooterSide side,
      Distance shooterToTargetDistance,
      Angle hoodAngle,
      AngularVelocity flywheelRpm) {
    return "/*<"
        + side.name()
        + " MAP>*/.addMeasurement(Meters.of("
        + shooterToTargetDistance.in(Meters)
        + "), Degrees.of("
        + hoodAngle.in(Degrees)
        + "), RPM.of("
        + flywheelRpm.in(RPM)
        + "));\n";
  }

  private void generateTableCodeImpl(AimController aimController) {
    AimingParams params = aimController.getCurrentParams();
    if (params == null) {
      return;
    }

    String code = "";
    code +=
        "// GENERATED AT TTD: Meters.of("
            + params.turret().turretToTargetDistance().in(Meters)
            + ") & HoodInterference."
            + params.hoodInterference().name()
            + "\n";
    code +=
        generateLookupTableCode(
            ShooterSide.LEFT,
            params.leftShooter().shooterToTargetDistance(),
            leftAngle(params),
            leftRpm(params));
    code +=
        generateLookupTableCode(
            ShooterSide.RIGHT,
            params.rightShooter().shooterToTargetDistance(),
            rightAngle(params),
            rightRpm(params));
    code += "\n";

    allGeneratedCode += code;
    Logger.recordOutput("Tuning/Shooters/GeneratedTableCode", allGeneratedCode);
  }

  public Command generateTableCode(AimController aimController) {
    return Commands.runOnce(() -> this.generateTableCodeImpl(aimController)).ignoringDisable(true);
  }

  private void setupHood(Hood hood, AimController aimController) {
    Function<AimingParams, Angle> hoodFn =
        switch (hood.getSide()) {
          case LEFT -> this::leftAngle;
          case RIGHT -> this::rightAngle;
        };

    RobotModeTriggers.test()
        .onTrue(
            Commands.runOnce(
                () -> {
                  tuningModeAlert.set(true);
                  hood.setDefaultCommand(
                      hood.run(
                          () -> {
                            Angle hoodAngle = hoodFn.apply(aimController.getCurrentParams());
                            if (hoodAngle != null) {
                              hood.setHoodAngle(hoodAngle);
                            }
                          }));
                }));
  }

  public ShootingTuner(AimController aimController, Hood leftHood, Hood rightHood) {
    setupHood(leftHood, aimController);
    setupHood(rightHood, aimController);
    Logger.recordOutput("Tuning/Shooters/GeneratedTableCode", allGeneratedCode);
  }

  private Command runFlywheel(Flywheel flywheel, AimController aimController) {
    Function<AimingParams, AngularVelocity> flywheelFn =
        switch (flywheel.getSide()) {
          case LEFT -> this::leftRpm;
          case RIGHT -> this::rightRpm;
        };

    return flywheel
        .run(
            () -> {
              AngularVelocity flywheelSpeed = flywheelFn.apply(aimController.getCurrentParams());
              if (flywheelSpeed != null) {
                flywheel.runVelocity(flywheelSpeed);
              } else {
                flywheel.runCoast();
              }
            })
        .finallyDo(() -> flywheel.runCoast());
  }

  public Command spinFlywheels(
      AimController aimController, Flywheel leftFlywheel, Flywheel rightFlywheel) {
    return Commands.parallel(
        runFlywheel(leftFlywheel, aimController), runFlywheel(rightFlywheel, aimController));
  }

  public Command shoot(
      AimController aimController,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Flywheel leftFlywheel,
      Flywheel rightFlywheel) {
    BooleanSupplier flywheelsAtGoal = leftFlywheel.atVelocity.and(rightFlywheel.atVelocity);

    return Commands.parallel(
        intake.intake(),
        Commands.sequence(
            Commands.waitUntil(flywheelsAtGoal),
            indexer.indexIntoShooter() // .onlyIf(aimingOkay).onlyWhile(aimingOkay).repeatedly(),
            ),
        spinFlywheels(aimController, leftFlywheel, rightFlywheel));
  }
}
