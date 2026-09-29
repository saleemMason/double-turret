package frc.robot.commands.aiming.functions;

import edu.wpi.first.units.measure.Distance;
import frc.robot.commands.aiming.AimCalculator.ShooterAimingParams;
import java.util.function.BooleanSupplier;

/** Chooses between two {@link ShotFunction}s at runtime, e.g. physics model vs measured table. */
public class SwitchableShotFunction implements ShotFunction {
  private final BooleanSupplier usePrimary;
  private final ShotFunction primary;
  private final ShotFunction fallback;

  public SwitchableShotFunction(
      BooleanSupplier usePrimary, ShotFunction primary, ShotFunction fallback) {
    this.usePrimary = usePrimary;
    this.primary = primary;
    this.fallback = fallback;
  }

  private ShotFunction active() {
    return usePrimary.getAsBoolean() ? primary : fallback;
  }

  @Override
  public double getBallTimeOfFlightSecs(Distance shooterToTargetDistance) {
    return active().getBallTimeOfFlightSecs(shooterToTargetDistance);
  }

  @Override
  public ShooterAimingParams getParams(Distance shooterToTargetDistance) {
    return active().getParams(shooterToTargetDistance);
  }
}
