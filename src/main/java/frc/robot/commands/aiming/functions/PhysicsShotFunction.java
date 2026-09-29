package frc.robot.commands.aiming.functions;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import frc.robot.commands.aiming.AimCalculator.ShooterAimingParams;

/**
 * A {@link ShotFunction} that computes hood angle, flywheel speed and time of flight from
 * projectile physics (see {@link ShotPhysics}) instead of looking them up in a measured table. It
 * can be dropped in anywhere an {@link InterpolatedShotFunction} is used.
 */
public class PhysicsShotFunction implements ShotFunction {
  private final ShotPhysics physics;

  public PhysicsShotFunction(ShotPhysics.Constants constants) {
    this.physics = new ShotPhysics(constants);
  }

  @Override
  public double getBallTimeOfFlightSecs(Distance shooterToTargetDistance) {
    return physics.solve(shooterToTargetDistance.in(Meters)).timeOfFlightSecs();
  }

  @Override
  public ShooterAimingParams getParams(Distance shooterToTargetDistance) {
    ShotPhysics.Result result = physics.solve(shooterToTargetDistance.in(Meters));
    return new ShooterAimingParams(
        Degrees.of(result.hoodDeg()), RPM.of(result.rpm()), shooterToTargetDistance);
  }
}
