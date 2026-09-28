package frc.robot.subsystems.shooters.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooters.ShooterSide;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final String name;
  private final ShooterSide side;

  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "{name}/GoalVelocity")
  private AngularVelocity goalVelocity = RadiansPerSecond.zero();

  private final AngularVelocity goalVelocityTolerance = RPM.of(100);

  @AutoLogOutput(key = "{name}/AtGoal")
  public final Trigger atGoal;

  @AutoLogOutput(key = "{name}/AtVelocity")
  public final Trigger atVelocity;

  public Flywheel(ShooterSide side, FlywheelIO flywheelIO) {
    super(side.getSubsystemName(Flywheel.class.getSimpleName()));
    this.name = getName();
    this.side = side;

    this.flywheelIO = flywheelIO;

    atGoal = new Trigger(() -> getVelocity().isNear(goalVelocity, goalVelocityTolerance));
    atVelocity = new Trigger(() -> goalVelocity.gt(RPM.one())).and(atGoal);
  }

  public ShooterSide getSide() {
    return side;
  }

  @AutoLogOutput(key = "{name}/MeasuredVelocity")
  private AngularVelocity getVelocity() {
    return flywheelInputs.velocity;
  }

  public void runCoast() {
    goalVelocity = RadiansPerSecond.zero();
    runVoltage(Volts.zero());
  }

  public void runVoltage(Voltage voltage) {
    goalVelocity = RadiansPerSecond.zero();
    this.flywheelIO.runVoltage(voltage);
  }

  public void runVelocity(AngularVelocity velocity) {
    goalVelocity = velocity;
    this.flywheelIO.runVelocity(goalVelocity);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs(name, flywheelInputs);
  }
}
