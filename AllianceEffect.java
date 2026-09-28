package frc.robot.subsystems.shooters;

public enum ShooterSide {
  LEFT,
  RIGHT;

  public String getSubsystemName(String subsystem) {
    return switch (this) {
      case LEFT -> "LeftShooter/" + subsystem;
      case RIGHT -> "RightShooter/" + subsystem;
    };
  }
}
