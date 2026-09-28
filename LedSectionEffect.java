package frc.robot.subsystems.shooters.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runClosedLoop(Angle position) {}
}
