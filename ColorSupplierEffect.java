package frc.robot.subsystems.shooters.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean mainConnected = false;
    public boolean followerConnected = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();

    public Voltage mainAppliedVoltage = Volts.zero();
    public Voltage followerAppliedVoltage = Volts.zero();

    public Current mainSupplyCurrent = Amps.zero();
    public Current followerSupplyCurrent = Amps.zero();

    public Temperature mainTemperature = Celsius.zero();
    public Temperature followerTemperature = Celsius.zero();
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runVelocity(AngularVelocity velocity) {}
}
