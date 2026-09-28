package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Subsystem that feeds balls from the hopper into the shooter */
public class Indexer extends SubsystemBase {
  public record IndexerConstants(Voltage indexerVoltage, Voltage kickerVoltage) {}

  private final IndexerConstants constants;

  private final RollerIO indexerIO;
  private final RollerIOInputsAutoLogged indexerInputs = new RollerIOInputsAutoLogged();

  private final RollerIO kickerIO;
  private final RollerIOInputsAutoLogged kickerInputs = new RollerIOInputsAutoLogged();

  public Indexer(IndexerConstants constants, RollerIO indexerIO, RollerIO kickerIO) {
    this.constants = constants;
    this.indexerIO = indexerIO;
    this.kickerIO = kickerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Indexer/Indexer", indexerInputs);

    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);
  }

  private void stopRollers() {
    indexerIO.stop();
    kickerIO.stop();
  }

  private void runForward() {
    indexerIO.runVoltage(constants.indexerVoltage);
    kickerIO.runVoltage(constants.kickerVoltage);
  }

  public Command indexIntoShooter() {
    return startEnd(this::runForward, this::stopRollers);
  }
}
