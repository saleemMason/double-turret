package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  public static record TurretConstants(
      Angle minAngle,
      Angle maxAngle,
      Angle trackOverlapMargin,
      Angle zeroOffset,
      Angle atGoalTolerance) {}

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private final int minAngleRotations;
  private final int maxAngleRotations;

  private final double minAngleRad;
  private final double maxAngleRad;

  private final double trackCenterRad;
  private final double minTrackAngleRad;
  private final double maxTrackAngleRad;

  private final Supplier<AngularVelocity> robotAngularVelocitySupplier;

  @AutoLogOutput(key = "Turret/GoalPosition")
  private Angle goalAngle = Radians.zero();

  private Angle zeroOffset;

  @AutoLogOutput(key = "Turret/AtGoal")
  public final Trigger atGoal;

  public Turret(
      TurretConstants constants,
      TurretIO turretIO,
      Supplier<AngularVelocity> robotAngularVelocitySupplier) {
    this.turretIO = turretIO;
    this.robotAngularVelocitySupplier = robotAngularVelocitySupplier;

    this.minAngleRotations = (int) Math.floor(constants.minAngle.in(Rotations));

    this.maxAngleRotations = (int) Math.ceil(constants.maxAngle.in(Rotations));

    this.minAngleRad = constants.minAngle.in(Radians);
    this.maxAngleRad = constants.maxAngle.in(Radians);

    this.zeroOffset = constants.zeroOffset;

    this.trackCenterRad = (minAngleRad + maxAngleRad) / 2.0;
    this.minTrackAngleRad =
        Math.max(trackCenterRad - Math.PI - constants.trackOverlapMargin.in(Radians), minAngleRad);
    this.maxTrackAngleRad =
        Math.min(trackCenterRad + Math.PI + constants.trackOverlapMargin.in(Radians), maxAngleRad);

    atGoal = new Trigger(() -> getPosition().isNear(goalAngle, constants.atGoalTolerance));
  }

  @AutoLogOutput(key = "Turret/MeasuredPosition")
  private Angle getPosition() {
    return turretInputs.position.plus(zeroOffset);
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocity")
  private AngularVelocity getVelocity() {
    return turretInputs.velocity;
  }

  /**
   * Aim the turret while tracking the target (not firing yet) Uses a restricted range of motion to
   * allow for a buffer when firing.
   *
   * @param rotation Robot relative turret rotation
   */
  public void setTrackingRotation(Angle rotation) {
    setRotation(rotation, true);
  }

  public void setFireRotation(Angle rotation) {
    setRotation(rotation, false);
  }

  private void setRotation(Angle rotation, boolean useTrackAngle) {
    double minTurretAngle = useTrackAngle ? minTrackAngleRad : minAngleRad;
    double maxTurretAngle = useTrackAngle ? maxTrackAngleRad : maxAngleRad;

    boolean hasBestSetpoint = false;
    double bestSetpoint = trackCenterRad;
    double turretPositionRad = getPosition().in(Radians);
    ArrayList<Double> possibleSetpoints = new ArrayList<>();

    for (int i = minAngleRotations; i <= maxAngleRotations; i++) {
      double potentialSetpoint = rotation.in(Radians) + (Math.PI * 2.0 * i);
      if (potentialSetpoint < minTurretAngle || potentialSetpoint > maxTurretAngle) {
        continue;
      }

      possibleSetpoints.add(potentialSetpoint);

      if (!hasBestSetpoint) {
        bestSetpoint = potentialSetpoint;
        hasBestSetpoint = true;

      } else if (Math.abs(potentialSetpoint - turretPositionRad)
          < Math.abs(bestSetpoint - turretPositionRad)) {
        bestSetpoint = potentialSetpoint;
      }
    }

    Logger.recordOutput(
        "Turret/PossibleSetpoints", possibleSetpoints.stream().mapToDouble((d) -> d).toArray());
    this.goalAngle = Radians.of(bestSetpoint);
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Turret", turretInputs);

    turretIO.setTurretRotation(goalAngle.minus(zeroOffset), robotAngularVelocitySupplier.get());
  }
}
