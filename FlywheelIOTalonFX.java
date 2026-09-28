package frc.robot.robots.compbot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.aiming.AimCalculator;
import frc.robot.commands.aiming.AimCalculator.AimingConstants;
import frc.robot.commands.aiming.AimCalculator.TurretLocationConstants;
import frc.robot.commands.aiming.functions.InterpolatedShotFunction;

public class CompBotAiming {
  private static final InterpolatedShotFunction hubLowerShotConstants =
      new InterpolatedShotFunction();
  private static final InterpolatedShotFunction hubUpperShotConstants =
      new InterpolatedShotFunction();

  static {
    // GENERATED AT TTD: Meters.of(1.6404964925770655) & HoodInterference.RIGHT_OVER_LEFT
    hubLowerShotConstants.addMeasurement(
        Meters.of(1.4503305628729235), Degrees.of(20.0), RPM.of(2750.0), Seconds.of(0.962));
    hubUpperShotConstants.addMeasurement(
        Meters.of(1.8307318871730007), Degrees.of(20.0), RPM.of(2900.0), Seconds.of(1.076));

    // GENERATED AT TTD: Meters.of(2.227950229130618) & HoodInterference.RIGHT_OVER_LEFT
    hubLowerShotConstants.addMeasurement(
        Meters.of(2.040156217682919), Degrees.of(20.0), RPM.of(3000.0), Seconds.of(1.13));
    hubUpperShotConstants.addMeasurement(
        Meters.of(2.4161679483049334), Degrees.of(25.0), RPM.of(3000.0), Seconds.of(1.12));

    // GENERATED AT TTD: Meters.of(3.5479330241547324) & HoodInterference.LEFT_OVER_RIGHT
    hubUpperShotConstants.addMeasurement(
        Meters.of(3.732547246358154), Degrees.of(28.0), RPM.of(3300.0), Seconds.of(1.206));
    hubLowerShotConstants.addMeasurement(
        Meters.of(3.3639751844327943), Degrees.of(34.0), RPM.of(3100.0), Seconds.of(1.162));

    // GENERATED AT TTD: Meters.of(5.196961959832727) & HoodInterference.LEFT_OVER_RIGHT
    hubUpperShotConstants.addMeasurement(
        Meters.of(5.385022500251461), Degrees.of(36.0), RPM.of(3700.0), Seconds.of(1.34876352194));
    hubLowerShotConstants.addMeasurement(
        Meters.of(5.00908578442038), Degrees.of(37.0), RPM.of(3700.0), Seconds.of(1.31786076329));
  }

  private static final TurretLocationConstants turretLocation =
      new TurretLocationConstants(
          new Transform2d(Inches.of(-4.25), Inches.zero(), Rotation2d.kZero),
          new Translation2d(Inches.zero(), Inches.of(7.5)),
          new Translation2d(Inches.zero(), Inches.of(-7.5)),
          Degrees.of(70));

  private static final AimingConstants hubAimingConstants =
      new AimingConstants(0.02, 10, hubLowerShotConstants, hubUpperShotConstants);

  public static AimCalculator getHubAimCalculator() {
    return new AimCalculator(turretLocation, hubAimingConstants);
  }

  private static final InterpolatedShotFunction passingShotConstants =
      new InterpolatedShotFunction();

  static {
    // GENERATED AT TTD: Meters.of(4.575447057039371) & HoodInterference.LEFT_OVER_RIGHT
    passingShotConstants.addMeasurement(
        Meters.of(4.575447057039371), Degrees.of(30.0), RPM.of(3600.0), Seconds.of(1.0));

    // GENERATED AT TTD: Meters.of(6.406914393586565) & HoodInterference.LEFT_OVER_RIGHT
    passingShotConstants.addMeasurement(
        Meters.of(6.406914393586565), Degrees.of(30.0), RPM.of(5600.0), Seconds.of(1.04));
  }

  private static final AimingConstants passingAimingConstants =
      new AimingConstants(0.02, 10, passingShotConstants, passingShotConstants);

  public static AimCalculator getPassingAimCalculator() {
    return new AimCalculator(turretLocation, passingAimingConstants);
  }
}
