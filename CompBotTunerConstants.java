package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.Resources;
import frc.robot.Robot.Subsystems;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoManager {
  public record AutoInputs(boolean mirrorAuto, double followDelay) {}

  private final AutoChooser<AutoInputs> autoChooser = new AutoChooser<>("Auto Chooser");
  private final LoggedNetworkBoolean mirrorAuto =
      new LoggedNetworkBoolean("SmartDashboard/Mirror Auto (Left to Right)", false);
  private final LoggedNetworkNumber followDelay =
      new LoggedNetworkNumber("SmartDashboard/Follower Auto Delay (Seconds)", 3.0);

  private final Subsystems subsystems;
  private final Resources resources;
  private final AutoDriver autoDriver;

  public AutoManager(Subsystems subsystems, Resources resources) {
    this.subsystems = subsystems;
    this.resources = resources;
    autoDriver = new AutoDriver(subsystems.drive(), subsystems.vision());
  }

  private AutoInputs getInputs() {
    return new AutoInputs(mirrorAuto.get(), followDelay.get());
  }

  /**
   * Update the auto manager and build the selected autos. This must be run every cycle to update
   * dashboard inputs and generate autos ahead of time
   */
  public void update() {
    autoChooser.update(getInputs());
  }

  /**
   * Returns the currently selected command.
   *
   * <p>If you plan on using this {@link Command} in a {@code Trigger} it is recommended to use
   * {@link #selectedCommandScheduler()} instead.
   *
   * @return The currently selected command.
   */
  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }

  /**
   * Gets a Command that schedules the selected auto routine. This Command shares the lifetime of
   * the scheduled Command. This Command can directly be bound to a trigger, like so:
   *
   * <pre><code>
   *     AutoManager autoManager = ...;
   *
   *     public Robot() {
   *         RobotModeTriggers.autonomous().whileTrue(autoManager.selectedCommandScheduler());
   *     }
   * </code></pre>
   *
   * @return A command that runs the selected auto
   */
  public Command selectedCommandScheduler() {
    return Commands.deferredProxy(() -> selectedCommand());
  }

  public void setupAutoRoutine(String name, RoutineSetup routineSetup) {
    autoChooser.addCommand(
        name,
        (inputs) -> {
          AutoRoutine routine = autoDriver.autoFactory.newRoutine(name);
          routineSetup.setupRoutine(routine, new AutoParameters(subsystems, resources, inputs));
          return routine.cmd();
        });
  }

  public void setupAutoCmd(String name, CommandFactory commandFactory) {
    autoChooser.addCommand(
        name,
        (inputs) -> commandFactory.buildCommand(new AutoParameters(subsystems, resources, inputs)));
  }

  public record AutoParameters(Subsystems s, Resources r, AutoInputs inputs) {}

  @FunctionalInterface
  public interface RoutineSetup {
    public void setupRoutine(AutoRoutine routine, AutoParameters p);
  }

  @FunctionalInterface
  public interface CommandFactory {
    public Command buildCommand(AutoParameters p);
  }
}
