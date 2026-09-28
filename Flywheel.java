package frc.robot.robots;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Mode;
import frc.robot.Robot.Subsystems;
import frc.robot.autos.AutoManager;
import frc.robot.commands.aiming.AimCalculator;
import frc.robot.robots.compbot.CompBotAiming;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.DeviceConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooters.ShooterSide;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOReplay;
import java.util.function.Supplier;

public abstract class RobotConfig {
  /**
   * Get the robot's subsystems.
   *
   * <p>Null subsystems may be provided to indicate the robot does not have the required hardware.
   *
   * @return The robot's subsystems
   */
  protected abstract Subsystems getSubsystems(Mode mode);

  private <S> S getSubsystemOrSim(S subsystemOrNull, Supplier<S> getSimSubsystem) {
    if (subsystemOrNull == null) {
      return getSimSubsystem.get();
    } else {
      return subsystemOrNull;
    }
  }

  /**
   * Get the robot's {@link Subsystems} Any null subsystems will be replaced with sim
   * implementations.
   *
   * @return The robot's subsystems or sim implementations
   */
  public Subsystems getSubsystemsOrSim(Mode mode) {
    Subsystems s = getSubsystems(mode);

    Drive drive = getSubsystemOrSim(s.drive(), () -> SimBotConfig.getSimDrive(mode));
    Vision vision = getSubsystemOrSim(s.vision(), () -> SimBotConfig.getSimVision(mode, drive));

    Intake intake = getSubsystemOrSim(s.intake(), () -> SimBotConfig.getSimIntake(mode));
    Indexer indexer = getSubsystemOrSim(s.indexer(), () -> SimBotConfig.getSimIndexer(mode));

    Turret turret = getSubsystemOrSim(s.turret(), () -> SimBotConfig.getSimTurret(mode, drive));
    Hood leftHood =
        getSubsystemOrSim(s.leftHood(), () -> SimBotConfig.getSimHood(mode, ShooterSide.LEFT));
    Flywheel leftFlywheel =
        getSubsystemOrSim(
            s.leftFlywheel(), () -> SimBotConfig.getSimFlywheel(mode, ShooterSide.LEFT));
    Hood rightHood =
        getSubsystemOrSim(s.rightHood(), () -> SimBotConfig.getSimHood(mode, ShooterSide.RIGHT));
    Flywheel rightFlywheel =
        getSubsystemOrSim(
            s.rightFlywheel(), () -> SimBotConfig.getSimFlywheel(mode, ShooterSide.RIGHT));

    return new Subsystems(
        drive, vision, intake, indexer, turret, leftHood, leftFlywheel, rightHood, rightFlywheel);
  }

  public AimCalculator getHubAimCalculator() {
    return CompBotAiming.getHubAimCalculator();
  }

  public AimCalculator getPassingAimCalculator() {
    return CompBotAiming.getPassingAimCalculator();
  }

  /**
   * Setup any robot-specific autos
   *
   * @param subsystems the robot's subsystems
   */
  public void setupAutos(AutoManager autos) {}

  public static Drive getStandardDrive(
      Mode mode, DriveConstants<TalonFXConfiguration, CANcoderConfiguration> constants) {
    DeviceConstants deviceConstants = constants.getDeviceConstants();

    return switch (mode) {
      case REAL -> new Drive(
          constants,
          new GyroIOPigeon2(constants.getDrivetrain(), deviceConstants),
          new ModuleIOTalonFX(constants.getFrontLeft(), deviceConstants),
          new ModuleIOTalonFX(constants.getFrontRight(), deviceConstants),
          new ModuleIOTalonFX(constants.getBackLeft(), deviceConstants),
          new ModuleIOTalonFX(constants.getBackRight(), deviceConstants));
      case SIM -> new Drive(
          constants,
          new GyroIO() {},
          new ModuleIOSim(constants.getFrontLeft()),
          new ModuleIOSim(constants.getFrontRight()),
          new ModuleIOSim(constants.getBackLeft()),
          new ModuleIOSim(constants.getBackRight()));
      default -> new Drive(
          constants,
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    };
  }

  public static VisionIO getLimelightVision(
      Mode mode, CameraConstants constants, Supplier<Pose2d> poseSupplier) {
    return switch (mode) {
      case REAL -> new VisionIOLimelight(constants, () -> poseSupplier.get().getRotation());
      case SIM -> new VisionIOPhotonVisionSim(constants, poseSupplier);
      default -> new VisionIOReplay(constants) {};
    };
  }

  public static VisionIO getPhotonVision(
      Mode mode, CameraConstants constants, Supplier<Pose2d> poseSupplier) {
    return switch (mode) {
      case REAL -> new VisionIOPhotonVision(constants);
      case SIM -> new VisionIOPhotonVisionSim(constants, poseSupplier);
      default -> new VisionIOReplay(constants) {};
    };
  }
}
