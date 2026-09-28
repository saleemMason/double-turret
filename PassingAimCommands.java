// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoManager;
import frc.robot.autos.routines.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotLocationTriggers;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.ShootingTuner;
import frc.robot.commands.TeleopCommands;
import frc.robot.commands.aiming.AimController;
import frc.robot.commands.aiming.PassingAimCommands;
import frc.robot.robots.RobotConfig;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.builtintest.BuiltInTest.Severity;
import frc.robot.util.builtintest.JoystickBIT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link
 * RobotRunner} periodic methods (other than the scheduler calls). Instead, the structure of the
 * robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class Robot {
  /**
   * Contains all the robot's "subsystems", classes that directly control mechanisms or hardware.
   */
  public static record Subsystems(
      Drive drive,
      Vision vision,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Hood leftHood,
      Flywheel leftFlywheel,
      Hood rightHood,
      Flywheel rightFlywheel) {}

  /**
   * Contains all the robot's "resources", classes that don't own any hardware but do
   * control/orchestrate other subsystems based on robot state.
   */
  public static record Resources(AimController aimController) {}

  private final Subsystems s;
  private final Resources r;

  private final AutoManager autoManager;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController overrideController = new CommandXboxController(1);

  @SuppressWarnings("unused")
  private final JoystickBIT driverTest =
      new JoystickBIT("driver", Severity.ERROR, driverController);

  @SuppressWarnings("unused")
  private final JoystickBIT overrideTest =
      new JoystickBIT("override", Severity.ERROR, overrideController);

  public Robot(RobotConfig robotConfig) {
    s = robotConfig.getSubsystemsOrSim(Constants.currentMode);

    RobotModeTriggers.teleop().onTrue(s.intake.deploy());

    AimController aimController =
        new AimController(
            robotConfig.getHubAimCalculator(),
            robotConfig.getPassingAimCalculator(),
            s.drive::getPose,
            s.drive::getChassisSpeeds);

    s.turret.setDefaultCommand(aimController.controlTurretTracking(s.turret));
    s.leftHood.setDefaultCommand(aimController.controlHoodSafe(s.leftHood));
    s.rightHood.setDefaultCommand(aimController.controlHoodSafe(s.rightHood));

    Trigger allianceZoneTrigger = RobotLocationTriggers.inOwnAllianceZone(s.drive::getPose);
    aimController.setDefaultCommand(
        aimController.aim(
            "setDefaultCommand",
            () -> {
              if (allianceZoneTrigger.getAsBoolean()) {
                return aimController.hub.getAimAtTargetParams(
                    ShootingCommands.getAllianceHubLocation());
              } else {
                return PassingAimCommands.getPassingAimingParams(aimController, s.drive.getPose());
              }
            }));

    r = new Resources(aimController);

    autoManager = new AutoManager(s, r);
    robotConfig.setupAutos(autoManager);
    setupAutos(autoManager);

    RobotModeTriggers.autonomous().whileTrue(autoManager.selectedCommandScheduler());

    configureDriverController();
    configureOverrideController();
  }

  private void setupAutos(AutoManager autoManager) {
    // DriveCharacterizationAutos.setupDriveCharacterizationAutos(autoManager);
    Autos.setupAutos(autoManager);
  }

  public void updateAutoManager() {
    autoManager.update();
  }

  private void configureDriverController() {
    // Default command, normal field-relative drive
    s.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            s.drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                s.drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(s.drive::stopWithX, s.drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .start()
        .onTrue(DriveCommands.resetGyro(s.drive, s.vision).ignoringDisable(true));

    driverController.rightTrigger().whileTrue(TeleopCommands.intake(s.intake));

    driverController
        .povUp()
        .and(RobotModeTriggers.teleop())
        .whileTrue(ShootingCommands.prepShot(r.aimController, s));

    driverController
        .leftTrigger()
        .and(RobotModeTriggers.teleop())
        .whileTrue(ShootingCommands.shoot(r.aimController, s));

    ShootingTuner shootingTuner = new ShootingTuner(r.aimController, s.leftHood, s.rightHood);
    driverController
        .back()
        .and(RobotModeTriggers.test())
        .onTrue(shootingTuner.generateTableCode(r.aimController));

    driverController
        .povUp()
        .and(RobotModeTriggers.test())
        .whileTrue(shootingTuner.spinFlywheels(r.aimController, s.leftFlywheel, s.rightFlywheel));

    driverController
        .leftTrigger()
        .and(RobotModeTriggers.test())
        .whileTrue(
            shootingTuner.shoot(
                r.aimController, s.intake, s.indexer, s.turret, s.leftFlywheel, s.rightFlywheel));
  }

  private void configureOverrideController() {
    Trigger shootingOverrideTrigger = overrideController.rightTrigger();

    shootingOverrideTrigger.and(overrideController.povUp()).onTrue(r.aimController.hoodFurther());
    shootingOverrideTrigger.and(overrideController.povDown()).onTrue(r.aimController.hoodCloser());

    shootingOverrideTrigger.and(overrideController.povLeft()).onTrue(r.aimController.turretLeft());
    shootingOverrideTrigger
        .and(overrideController.povRight())
        .onTrue(r.aimController.turretRight());

    shootingOverrideTrigger.and(overrideController.a()).onTrue(r.aimController.resetOffsets());
    shootingOverrideTrigger
        .and(overrideController.y())
        .toggleOnTrue(
            r.aimController
                .aimStaticShot(Degrees.of(25), RPM.of(3000))
                .alongWith(s.vision.disablePoseUpdatedCmd())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    shootingOverrideTrigger
        .and(overrideController.x())
        .onTrue(ShootingCommands.zeroHoods(s.leftHood, s.rightHood));

    overrideController
        .povDown()
        .and(shootingOverrideTrigger.negate())
        .whileTrue(s.intake().outake());
  }
}
