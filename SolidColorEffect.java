package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretRotation(Angle position, AngularVelocity robotAngularVelocity) {}

  public default void setTurretRotation(Angle position) {}
}
