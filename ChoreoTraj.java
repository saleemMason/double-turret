package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser<I> {
  private final String doNothingName = "Nothing";

  private final Map<String, AutoBuildFunction<I>> autos = new HashMap<>();
  private final LoggedDashboardChooser<String> autoChooser;

  private Optional<Alliance> allianceAtGeneration = Optional.empty();
  private I inputsAtGeneration = null;
  private Command generatedCommand = Commands.none();

  public AutoChooser(String key) {
    autoChooser = new LoggedDashboardChooser<>(key);
    addCommandImpl(doNothingName, (inputs) -> Commands.none());
    autoChooser.onChange((selected) -> update(inputsAtGeneration));
  }

  /**
   * Update the auto chooser and regenerate autos if necessary
   *
   * @param inputs The inputs at the current time
   */
  public void update(I inputs) {
    // Don't build autos while enabled
    if (DriverStation.isEnabled()) {
      return;
    }

    String selected = autoChooser.get();
    if (generatedCommand.getName().equals(selected)
        && allianceAtGeneration.equals(DriverStation.getAlliance())
        && inputsAtGeneration != null
        && inputsAtGeneration.equals(inputs)) {
      // early return if the selected auto matches the generated auto
      return;
    }

    allianceAtGeneration = DriverStation.getAlliance();
    if (allianceAtGeneration.isEmpty() || !autos.containsKey(selected)) {
      selected = doNothingName;
    }

    inputsAtGeneration = inputs;
    generatedCommand = autos.get(selected).buildAuto(inputsAtGeneration).withName(selected);
  }

  public void addCommand(String name, AutoBuildFunction<I> builder) {
    if (name.equals(doNothingName)) {
      throw new IllegalArgumentException(
          "Auto name may not be equal to the default do-nothing name: '" + doNothingName + "'");
    }

    addCommandImpl(name, builder);
  }

  private void addCommandImpl(String name, AutoBuildFunction<I> builder) {
    autoChooser.addOption(name, name);
    autos.put(name, builder);
  }

  public Command selectedCommand() {
    return generatedCommand;
  }

  @FunctionalInterface
  public static interface AutoBuildFunction<I> {
    public Command buildAuto(I inputs);
  }
}
