package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConstants;

public class VisionIOReplay implements VisionIO {
  private final CameraConstants cameraConstants;

  public VisionIOReplay(CameraConstants cameraConstants) {
    this.cameraConstants = cameraConstants;
  }

  @Override
  public CameraConstants getCameraConstants() {
    return this.cameraConstants;
  }
}
