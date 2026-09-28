package frc.robot.subsystems.shooters.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooters.ShooterSide;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public record HoodConstants(ShooterSide side, Angle minAngle, Angle trenchSafeAngle) {}

  private final HoodConstants constants;
  private final String name;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final MutAngle zeroAngle;

  private final Time zeroDebounceTime = Seconds.of(1);
  private final Voltage zeroVoltage = Volts.of(-2);
  private final Timer zeroTimer = new Timer();

  @AutoLogOutput(key = "{name}/GoalPosition")
  private Angle goalAngle = Radians.zero();

  @AutoLogOutput(key = "{name}/AtGoal")
  public final Trigger atGoal;

  // Triggers when either `atGoal` or if the encoder is disconnected
  @AutoLogOutput(key = "{name}/AtGoal")
  public final Trigger readyToShoot;

  @AutoLogOutput(key = "{name}/AtSafeAngle")
  public final Trigger atSafeAngle;

  public Hood(HoodConstants constants, HoodIO hoodIO) {
    super(constants.side.getSubsystemName(Hood.class.getSimpleName()));
    this.constants = constants;
    this.name = getName();
    this.zeroAngle = constants.minAngle.mutableCopy().mut_times(-1);
    goalAngle = zeroAngle;

    this.hoodIO = hoodIO;

    atGoal = new Trigger(() -> getPosition().isNear(goalAngle, Degrees.one()));
    atSafeAngle = new Trigger(() -> isHoodAngleSafe(getPosition()));
    readyToShoot = atGoal.or(() -> !hoodInputs.motorConnected);
  }

  public ShooterSide getSide() {
    return constants.side;
  }

  @AutoLogOutput(key = "{name}/MeasuredPosition")
  private Angle getPosition() {
    return hoodInputs.position.minus(zeroAngle);
  }

  @AutoLogOutput(key = "{name}/MeasuredVelocity")
  private AngularVelocity getVelocity() {
    return hoodInputs.velocity;
  }

  public void setHoodAngle(Angle hoodAngle) {
    goalAngle = hoodAngle;
    hoodIO.runClosedLoop(goalAngle.plus(zeroAngle));
  }

  /** Get if the provided hood angle is safe. */
  public boolean isHoodAngleSafe(Angle hoodAngle) {
    return hoodAngle.lt(constants.trenchSafeAngle.plus(Degrees.one()));
  }

  /** Returns the provided angle if it is safe, otherwise returns the max safe angle */
  public Angle getAngleOrSafeAngle(Angle hoodAngle) {
    if (hoodAngle.gt(constants.trenchSafeAngle)) {
      return constants.trenchSafeAngle;
    } else {
      return hoodAngle;
    }
  }

  public void lowerForTrench() {
    setHoodAngle(constants.trenchSafeAngle);
  }

  private void setZeroPosition() {
    zeroAngle.mut_replace(hoodInputs.position.minus(constants.minAngle));
  }

  public Command zeroHood() {
    return new FunctionalCommand(
        () -> {
          zeroTimer.restart();
        },
        () -> {
          hoodIO.runVoltage(zeroVoltage);
        },
        (interrupted) -> {
          hoodIO.runVoltage(Volts.zero());
          if (!interrupted) {
            setZeroPosition();
          }
        },
        () ->
            hoodInputs.velocity.isNear(RadiansPerSecond.zero(), RadiansPerSecond.of(1e-3))
                && zeroTimer.hasElapsed(zeroDebounceTime),
        this);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs(name, hoodInputs);

    if (DriverStation.isDisabled()) {
      if (hoodInputs.position.lt(zeroAngle.plus(constants.minAngle))) {
        setZeroPosition();
      }
    }
  }
}
