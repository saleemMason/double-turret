package frc.robot.commands.aiming.functions;

import edu.wpi.first.units.measure.Distance;
import frc.robot.commands.aiming.AimCalculator.ShooterAimingParams;

public interface ShotFunction {
  double getBallTimeOfFlightSecs(Distance shooterToTargetDistance);

  ShooterAimingParams getParams(Distance shooterToTargetDistance);
}
