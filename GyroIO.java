package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.intake.Intake;

public class TeleopCommands {
  public static Command intake(Intake intake) {
    return intake.intake().asProxy().repeatedly().until(RobotModeTriggers.disabled());
  }
}
