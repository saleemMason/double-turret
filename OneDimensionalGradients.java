package frc.robot.subsystems.intake.pivot;

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
public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean motorConnected = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void runNeutral() {}

  public default void runVoltage(Voltage voltage) {}

  public default void setPosition(Angle position) {}

  public default void resetZero(Angle position) {}
}
