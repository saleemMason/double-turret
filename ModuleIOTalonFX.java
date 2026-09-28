package frc.robot.commands.aiming.functions;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.commands.aiming.AimCalculator.ShooterAimingParams;

public class InterpolatedShotFunction implements ShotFunction {
  final InterpolatingDoubleTreeMap hoodAngleRadMap;
  final InterpolatingDoubleTreeMap flywheelSpeedRadPerSecMap;
  final InterpolatingDoubleTreeMap ballTimeOfFlightSecMap;

  public InterpolatedShotFunction() {
    this.hoodAngleRadMap = new InterpolatingDoubleTreeMap();
    this.flywheelSpeedRadPerSecMap = new InterpolatingDoubleTreeMap();
    this.ballTimeOfFlightSecMap = new InterpolatingDoubleTreeMap();
  }

  public void addMeasurement(
      Distance distance, Angle hoodAngle, AngularVelocity flywheelSpeed, Time ballTimeOfFlight) {
    double distanceMeters = distance.in(Meters);
    hoodAngleRadMap.put(distanceMeters, hoodAngle.in(Radians));
    flywheelSpeedRadPerSecMap.put(distanceMeters, flywheelSpeed.in(RadiansPerSecond));
    ballTimeOfFlightSecMap.put(distanceMeters, ballTimeOfFlight.in(Seconds));
  }

  /* Use method with TOF for shoot on the move */
  public void addMeasurement(Distance distance, Angle hoodAngle, AngularVelocity flywheelSpeed) {
    addMeasurement(distance, hoodAngle, flywheelSpeed, Seconds.zero());
  }

  @Override
  public double getBallTimeOfFlightSecs(Distance shooterToTargetDistance) {
    return ballTimeOfFlightSecMap.get(shooterToTargetDistance.in(Meters));
  }

  @Override
  public ShooterAimingParams getParams(Distance shooterToTargetDistance) {
    double shooterToTargetDistanceMeters = shooterToTargetDistance.in(Meters);

    return new ShooterAimingParams(
        Radians.of(hoodAngleRadMap.get(shooterToTargetDistanceMeters)),
        RadiansPerSecond.of(flywheelSpeedRadPerSecMap.get(shooterToTargetDistanceMeters)),
        shooterToTargetDistance);
  }
}
