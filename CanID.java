package frc.robot.subsystems.generic.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean objectDetected = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}
}
