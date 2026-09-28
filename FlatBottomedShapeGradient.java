package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public record IntakeConstants(
      Voltage intakeVoltage, Angle pivotStowAngle, Angle pivotDeployAngle) {}

  private final IntakeConstants constants;

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private Angle deployFinishedTolerance = Degrees.of(5);
  private Time deployZeroDebounce = Seconds.of(2);
  private Timer deployZeroTimer = new Timer();
  private boolean runningDeploy = false;

  public Intake(IntakeConstants constants, RollerIO rollerIO, PivotIO pivotIO) {
    this.constants = constants;
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);

    if (DriverStation.isDisabled()) {
      pivotIO.runNeutral();
    }

    if (runningDeploy) {
      if (pivotInputs.position.isNear(constants.pivotDeployAngle, deployFinishedTolerance)) {
        runningDeploy = false;
        pivotIO.runNeutral();
      } else if (pivotInputs.velocity.isEquivalent(RadiansPerSecond.zero())
          && deployZeroTimer.hasElapsed(deployZeroDebounce)) {
        runningDeploy = false;
        pivotIO.runNeutral();
        pivotIO.resetZero(constants.pivotDeployAngle);
      }
    }
  }

  private void stopRollers() {
    rollerIO.stop();
  }

  private void intakeRollers() {
    rollerIO.runVoltage(constants.intakeVoltage);
  }

  private void outakeRollers() {
    rollerIO.runVoltage(constants.intakeVoltage.unaryMinus());
  }

  public Command intake() {
    return runEnd(this::intakeRollers, this::stopRollers);
  }

  public Command outake() {
    return runEnd(this::outakeRollers, this::stopRollers);
  }

  public Command deploy() {
    return Commands.runOnce(
        () -> {
          pivotIO.setPosition(constants.pivotDeployAngle);
          deployZeroTimer.restart();
          runningDeploy = true;
        });
  }

  /*public Command stow() {
    return Commands.runOnce(() -> pivotIO.setPosition(constants.pivotStowAngle));
  }*/
}
