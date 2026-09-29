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
import frc.robot.commands.aiming.functions.PhysicsShotFunction;
import frc.robot.commands.aiming.functions.ShotPhysics;
import frc.robot.commands.aiming.functions.SwitchableShotFunction;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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

  // ---------------------------------------------------------------------------------------------
  // Physics-based hub shot (optional). Chosen at runtime with the "Use Physics Shot" dashboard
  // toggle; the measured tables above stay as the default and the fallback.
  //
  // The shot aims for the lowest-energy trajectory that still drops into the hub at a steep enough
  // angle (about 45 degrees below horizontal) to leave a wide scoring window. The exit speed is the
  // flywheel surface speed times exitSpeedRatio; the ratio and the hood-to-launch-angle fit
  // (launchAngle = intercept + slope * hoodAngle) were fitted to the measured tables above.
  //
  // The fits only cover hood angles up to about 37 degrees. The hood can physically reach 50
  // degrees, but angles beyond the measured range are extrapolated, so the hood is capped at
  // hubMaxHoodDeg. Raise it in small steps and check real shots before going further; the solver
  // uses a steeper (higher-energy) shot whenever the hood can't reach the flatter one.
  // ---------------------------------------------------------------------------------------------
  private static final double hubMaxHoodDeg = 38.0;

  private static ShotPhysics.Constants hubPhysicsConstants(
      double exitSpeedRatio, double hoodFitInterceptDeg, double hoodFitSlope) {
    return new ShotPhysics.Constants(
        Inches.of(16.0).in(Meters), // ball height leaving the shooter
        Inches.of(72.0).in(Meters), // hub opening height
        Inches.of(4.0).in(Meters), // flywheel diameter
        exitSpeedRatio,
        45.0, // preferred entry angle into the hub, degrees below horizontal
        80.0, // steepest entry angle the solver may fall back to
        hoodFitInterceptDeg,
        hoodFitSlope,
        20.0, // minimum hood angle (same convention as hoodConstants in CompBotConfig)
        hubMaxHoodDeg,
        4200.0); // never command more than this flywheel RPM
  }

  private static final PhysicsShotFunction hubLowerPhysicsShot =
      new PhysicsShotFunction(hubPhysicsConstants(0.440, 89.56, -0.686));
  private static final PhysicsShotFunction hubUpperPhysicsShot =
      new PhysicsShotFunction(hubPhysicsConstants(0.442, 92.18, -0.844));

  public static AimCalculator getHubAimCalculator() {
    LoggedNetworkBoolean usePhysicsShot =
        new LoggedNetworkBoolean("SmartDashboard/Use Physics Shot", false);
    AimingConstants hubAimingConstants =
        new AimingConstants(
            0.02,
            10,
            new SwitchableShotFunction(
                usePhysicsShot::get, hubLowerPhysicsShot, hubLowerShotConstants),
            new SwitchableShotFunction(
                usePhysicsShot::get, hubUpperPhysicsShot, hubUpperShotConstants));
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
