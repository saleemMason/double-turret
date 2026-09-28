package frc.robot.subsystems.generic.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIODio implements BeamBreakIO {
  private final DigitalInput sensor;

  public BeamBreakIODio(int dioId) {
    this.sensor = new DigitalInput(dioId);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.objectDetected = !sensor.get();
  }
}
