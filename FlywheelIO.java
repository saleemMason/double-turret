package frc.robot.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.Mode;
import frc.robot.Robot.Subsystems;
import frc.robot.robots.compbot.CompBotDriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.pivot.PivotIOSim.PivotConstantsSim;
import frc.robot.subsystems.shooters.ShooterSide;
import frc.robot.subsystems.shooters.flywheel.Flywheel;
import frc.robot.subsystems.shooters.flywheel.FlywheelIO;
import frc.robot.subsystems.shooters.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooters.hood.Hood;
import frc.robot.subsystems.shooters.hood.Hood.HoodConstants;
import frc.robot.subsystems.shooters.hood.HoodIO;
import frc.robot.subsystems.shooters.hood.HoodIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.pid.PidConstants;

/**
 * Base config for a purely simulated robot. Individual subsystems can be taken from this class to
 * allow robots without the necessary hardware to run the rest of the robot code.
 */
public class SimBotConfig extends RobotConfig {
  private static final DriveConstants<TalonFXConfiguration, CANcoderConfiguration>
      simDriveConstants = new CompBotDriveConstants();

  private static final VisionConstants simVisionConstants = new VisionConstants();

  private static final PivotConstantsSim simIntakePivotConstants =
      new PivotConstantsSim(
          DCMotor.getKrakenX60(1),
          15,
          Pound.of(10),
          Feet.of(1),
          Degrees.of(-15),
          Degrees.of(90),
          Degrees.of(90),
          new PidConstants().withKP(10));

  private static final IntakeConstants simIntakeConstants =
      new IntakeConstants(
          Volts.of(12), simIntakePivotConstants.maxAngle(), simIntakePivotConstants.minAngle());

  private static final IndexerConstants simIndexerConstants =
      new IndexerConstants(Volts.of(12), Volts.of(12));

  private static final TurretConstants simTurretConstants =
      new TurretConstants(
          Degrees.of(-210.0), Degrees.of(210.0), Degrees.of(10), Degrees.of(0), Degrees.of(5));

  public static Drive getSimDrive(Mode mode) {
    return getStandardDrive(mode.forceSim(), simDriveConstants);
  }

  public static Vision getSimVision(Mode mode, Drive drive) {
    return new Vision(simVisionConstants, drive::addVisionMeasurement);
  }

  public static RollerIO getSimRoller(Mode mode) {
    return switch (mode.forceSim()) {
      case SIM -> new RollerIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.01);
      default -> new RollerIO() {};
    };
  }

  public static PivotIO getSimIntakePivot(Mode mode) {
    return switch (mode.forceSim()) {
      case SIM -> new PivotIOSim(simIntakePivotConstants);
      default -> new PivotIO() {};
    };
  }

  public static Intake getSimIntake(Mode mode) {
    return new Intake(simIntakeConstants, getSimRoller(mode), getSimIntakePivot(mode));
  }

  public static Indexer getSimIndexer(Mode mode) {
    return new Indexer(simIndexerConstants, getSimRoller(mode), getSimRoller(mode));
  }

  public static Turret getSimTurret(Mode mode, Drive drive) {
    return new Turret(
        simTurretConstants,
        switch (mode.forceSim()) {
          case SIM -> new TurretIOSim(
              DCMotor.getKrakenX60Foc(1), 1.0, 0.001, new PidConstants().withKP(1.0));
          default -> new TurretIO() {};
        },
        () -> RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));
  }

  public static Hood getSimHood(Mode mode, ShooterSide side) {
    return new Hood(
        new HoodConstants(side, Degrees.zero(), Degrees.zero()),
        switch (mode.forceSim()) {
          case SIM -> new HoodIOSim(
              DCMotor.getKrakenX44(1), 20.0, 0.001, new PidConstants().withKP(1.0));
          default -> new HoodIO() {};
        });
  }

  public static Flywheel getSimFlywheel(Mode mode, ShooterSide side) {
    return new Flywheel(
        side,
        switch (mode.forceSim()) {
          case SIM -> new FlywheelIOSim(
              DCMotor.getKrakenX60(2), 1.0, 1.0, new PidConstants().withKP(1.0));
          default -> new FlywheelIO() {};
        });
  }

  @Override
  public Subsystems getSubsystems(Mode mode) {
    return new Subsystems(null, null, null, null, null, null, null, null, null);
  }
}
