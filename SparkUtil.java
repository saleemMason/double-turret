package frc.robot.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for a "roller" type mechanism who's only effects are to spin freely at some voltage
 * or current
 */
public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean motorConnected = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runTorque(Current current) {}

  public default void stop() {
    runVoltage(Volts.zero());
  }
}
