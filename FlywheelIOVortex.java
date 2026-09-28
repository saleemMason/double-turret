package frc.robot.robots.compbot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.Mode;
import frc.robot.Robot.Subsystems;
import frc.robot.robots.RobotConfig;
import frc.robot.robots.SimBotConfig;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIODoubleTalonFX;
import frc.robot.subsystems.generic.roller.RollerIODoubleTalonFX.RollerConstantsDoubleTalonFX;
import frc.robot.subsystems.generic.roller.RollerIOSim;
import frc.robot.subsystems.generic.roller.RollerIOTalonFX;
import frc.robot.subsystems.generic.roller.RollerIOTalonFX.RollerConstantsTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.pivot.PivotIOSim.PivotConstantsSim;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX.PivotConstantsTalonFX;
import frc.robot.subsystems.shooters.ShooterSide;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.flywheel.FlywheelIO;
import frc.robot.subsystems.shooters.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooters.flywheel.FlywheelIOVortex;
import frc.robot.subsystems.shooters.flywheel.FlywheelIOVortex.FlywheelConstantsVortex;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.shooters.hood.Hood.HoodConstants;
import frc.robot.subsystems.shooters.hood.HoodIO;
import frc.robot.subsystems.shooters.hood.HoodIOServo;
import frc.robot.subsystems.shooters.hood.HoodIOServo.HoodConstantsServo;
import frc.robot.subsystems.shooters.hood.HoodIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretIOTalonFX.TurretConstantsTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.robot.util.CanID;
import frc.robot.util.ServoManager;
import frc.robot.util.ServoManager.ServoConfig;
import frc.robot.util.pid.ClosedLoopConstants;
import frc.robot.util.pid.ClosedLoopConstants.GravityFeedforwardType;
import frc.robot.util.pid.MotionProfileConstants;
import frc.robot.util.pid.PidConstants;

public class CompBotConfig extends RobotConfig {
  private static final CompBotDriveConstants driveConstants = new CompBotDriveConstants();

  private static final CANBus canivoreBus = driveConstants.getCanBus();

  private static final VisionConstants visionConstants = new VisionConstants();

  private static final CameraConstants leftCamera =
      new CameraConstants(
          "limelight-c",
          new Transform3d(
              Inches.of(-5),
              Inches.of(13.5),
              Inches.of(9.5),
              new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.of(90))));

  private static final CameraConstants centerCamera =
      new CameraConstants(
          "limelight-b",
          new Transform3d(
              Inches.of(-11.5),
              Inches.of(0),
              Inches.of(15.125),
              new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(180))));

  private static final CameraConstants rightCamera =
      new CameraConstants(
          "limelight-a",
          new Transform3d(
              Inches.of(-5),
              Inches.of(-13.5),
              Inches.of(9.5),
              new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.of(-90))));

  private static final RollerConstantsDoubleTalonFX intakeRoller =
      new RollerConstantsDoubleTalonFX(new CanID(21, canivoreBus), new CanID(22, canivoreBus)) {
        {
          currentLimit = Amps.of(20);
          invertedValue = InvertedValue.CounterClockwise_Positive;
          neutralMode = NeutralModeValue.Coast;
        }
      };

  private static final PivotConstantsTalonFX intakePivotMotor =
      new PivotConstantsTalonFX(
          new CanID(20, canivoreBus),
          (48.0 / 8.0) * (24.0 / 15.0) * (42.0 / 18.0),
          InvertedValue.CounterClockwise_Positive,
          Amps.of(30),
          Rotations.of(-0.0),
          Rotations.of(0.45),
          Rotations.of(0.45),
          new ClosedLoopConstants()
              .withKP(0.0)
              .withKD(10.0)
              .withKA(10.0)
              .withKS(5.0)
              .withKG(30.0)
              .withGravityType(GravityFeedforwardType.ARM_COSINE),
          new MotionProfileConstants(
              RotationsPerSecond.of(5.0), RotationsPerSecondPerSecond.of(3.0)));

  private static final IntakeConstants intakeConstants =
      new IntakeConstants(Volts.of(8), intakePivotMotor.maxAngle(), intakePivotMotor.minAngle());

  private static final RollerConstantsDoubleTalonFX indexerRoller =
      new RollerConstantsDoubleTalonFX(new CanID(30, canivoreBus), new CanID(31, canivoreBus)) {
        {
          invertedValue = InvertedValue.Clockwise_Positive;
          neutralMode = NeutralModeValue.Brake;
          currentLimit = Amps.of(20);
        }
      };

  private static final RollerConstantsTalonFX kickerRoller =
      new RollerConstantsTalonFX(new CanID(32, canivoreBus)) {
        {
          neutralMode = NeutralModeValue.Coast;
          invertedValue = InvertedValue.Clockwise_Positive;
          currentLimit = Amps.of(30);
          useFoc = false;
        }
      };

  private static final IndexerConstants indexerConstants =
      new IndexerConstants(Volts.of(10), Volts.of(7));

  private static final Angle turretMinAngle = Rotations.of(-1.0);
  private static final Angle turretMaxAngle = Rotations.of(1.0);
  private static final Angle turretZeroAngle = Degrees.of(180);

  private static final Angle turretTrackOverlap = Degrees.of(35);
  private static final Angle turretAtGoalTolerance = Degrees.of(10);

  private static final TurretConstantsTalonFX turretMotor =
      new TurretConstantsTalonFX(
          new CanID(40, canivoreBus),
          (132.0 / 24.0) * (48.0 / 18.0) * (32.0 / 8.0),
          InvertedValue.CounterClockwise_Positive,
          turretMinAngle,
          turretMaxAngle,
          new ClosedLoopConstants().withKP(3500).withKD(32),
          new MotionProfileConstants(RotationsPerSecond.of(3), RotationsPerSecondPerSecond.of(4)));

  private static final TurretConstants turretConstants =
      new TurretConstants(
          turretMinAngle.plus(turretZeroAngle),
          turretMaxAngle.plus(turretZeroAngle),
          turretTrackOverlap,
          turretZeroAngle,
          turretAtGoalTolerance);

  private static final int servoHubDeviceId = 45;
  private static final ServoChannel.ChannelId leftHoodServoChannel =
      ServoChannel.ChannelId.kChannelId1;
  private static final ServoChannel.ChannelId rightHoodServoChannel =
      ServoChannel.ChannelId.kChannelId0;

  private static final HoodConstantsServo leftHoodServo =
      new HoodConstantsServo(
          new CanID(41),
          true,
          334.0 / 16.0,
          new MagnetSensorConfigs()
              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
              .withMagnetOffset(-0.360107421875),
          new PidConstants().withKP(15));

  private static final HoodConstants leftHoodConstants =
      new HoodConstants(ShooterSide.LEFT, Degrees.of(20), Degrees.of(32));

  private static final HoodConstantsServo rightHoodServo =
      new HoodConstantsServo(
          new CanID(43),
          leftHoodServo.servoInverted(),
          leftHoodServo.sensorToMechanismReduction(),
          new MagnetSensorConfigs()
              .withSensorDirection(leftHoodServo.encoderConfigs().SensorDirection)
              .withMagnetOffset(0.13720703125),
          leftHoodServo.pid());

  private static final HoodConstants rightHoodConstants =
      new HoodConstants(
          ShooterSide.RIGHT, leftHoodConstants.minAngle(), leftHoodConstants.trenchSafeAngle());

  private static final FlywheelConstantsVortex leftFlywheelMotors =
      new FlywheelConstantsVortex(
          41, 42, new ClosedLoopConstants().withKV(0.0017699).withKP(0.00012));

  private static final FlywheelConstantsVortex rightFlywheelMotors =
      new FlywheelConstantsVortex(44, 43, new ClosedLoopConstants().withKV(0.0018).withKP(0.00015));

  private static Hood getHood(Mode mode, ShooterSide side, ServoManager servoManager) {
    return new Hood(
        switch (side) {
          case LEFT -> leftHoodConstants;
          case RIGHT -> rightHoodConstants;
        },
        switch (mode) {
          case REAL -> new HoodIOServo(
              switch (side) {
                case LEFT -> leftHoodServo;
                case RIGHT -> rightHoodServo;
              },
              servoManager.getContinuousServo(
                  switch (side) {
                    case LEFT -> leftHoodServoChannel;
                    case RIGHT -> rightHoodServoChannel;
                  },
                  ServoConfig.SRS_V2_CONTINUOUS));
          case SIM -> new HoodIOSim(
              DCMotor.getKrakenX44(1), 1.0, 0.01, new PidConstants().withKP(1.0));
          default -> new HoodIO() {};
        });
  }

  private static Flywheel getFlywheel(Mode mode, ShooterSide side) {
    return new Flywheel(
        side,
        switch (mode) {
          case REAL -> new FlywheelIOVortex(
              switch (side) {
                case LEFT -> leftFlywheelMotors;
                case RIGHT -> rightFlywheelMotors;
              });
          case SIM -> new FlywheelIOSim(
              DCMotor.getNeoVortex(2), 1.0, 1.0, new PidConstants().withKP(1.0));
          default -> new FlywheelIO() {};
        });
  }

  @Override
  public Subsystems getSubsystems(Mode mode) {
    Drive drive = getStandardDrive(mode, driveConstants);

    Vision vision =
        new Vision(
            visionConstants,
            drive::addVisionMeasurement,
            getLimelightVision(mode, leftCamera, drive::getPose),
            getLimelightVision(mode, centerCamera, drive::getPose),
            getLimelightVision(mode, rightCamera, drive::getPose));

    /*Questnav questnav = getQuestnav(mode, questnavConstants, drive);*/

    Intake intake =
        switch (mode) {
          case REAL -> new Intake(
              intakeConstants,
              new RollerIODoubleTalonFX(intakeRoller),
              SimBotConfig.getSimIntakePivot(mode)); // new PivotIOTalonFX(intakePivotMotor));
          case SIM -> new Intake(
              intakeConstants,
              new RollerIOSim(DCMotor.getKrakenX60Foc(2), intakeRoller.motorReduction, 0.01),
              new PivotIOSim(
                  new PivotConstantsSim(
                      DCMotor.getKrakenX44Foc(1),
                      intakePivotMotor.motorReduction(),
                      Pounds.of(10),
                      Inches.of(11),
                      intakePivotMotor.minAngle(),
                      intakePivotMotor.maxAngle(),
                      intakePivotMotor.startingAngle(),
                      new PidConstants().withKP(10))));
          default -> new Intake(intakeConstants, new RollerIO() {}, new PivotIO() {});
        };

    Indexer indexer =
        switch (mode) {
          case REAL -> new Indexer(
              indexerConstants,
              new RollerIODoubleTalonFX(indexerRoller),
              new RollerIOTalonFX(kickerRoller));
          case SIM -> new Indexer(
              indexerConstants,
              new RollerIOSim(DCMotor.getKrakenX60Foc(2), indexerRoller.motorReduction, 0.01),
              new RollerIOSim(DCMotor.getKrakenX60Foc(1), kickerRoller.motorReduction, 0.01));
          default -> new Indexer(indexerConstants, new RollerIO() {}, new RollerIO() {});
        };

    Turret turret =
        new Turret(
            turretConstants,
            switch (mode) {
              case REAL -> new TurretIOTalonFX(turretMotor);
              case SIM -> new TurretIOSim(
                  DCMotor.getKrakenX60Foc(1),
                  turretMotor.motorReduction(),
                  1.0,
                  new PidConstants().withKP(10));
              default -> new TurretIO() {};
            },
            () -> RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));

    ServoManager servoManager =
        switch (mode) {
          case REAL -> new ServoManager(servoHubDeviceId);
          default -> null;
        };

    Hood leftHood = getHood(mode, ShooterSide.LEFT, servoManager);
    Flywheel leftFlywheel = getFlywheel(mode, ShooterSide.LEFT);
    Hood rightHood = getHood(mode, ShooterSide.RIGHT, servoManager);
    Flywheel rightFlywheel = getFlywheel(mode, ShooterSide.RIGHT);

    return new Subsystems(
        drive, vision, intake, indexer, turret, leftHood, leftFlywheel, rightHood, rightFlywheel);
  }
}
