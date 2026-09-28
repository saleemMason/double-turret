package frc.robot;

import frc.robot.robots.RobotConfig;
import frc.robot.robots.SimBotConfig;
import frc.robot.robots.compbot.CompBotConfig;
import frc.robot.util.robotidentifter.RobotIdentification;
import java.util.List;

public enum RobotId implements RobotIdentification {
  COMPBOT(new CompBotConfig(), "D9-D8-C2-6D-0B-12"),
  SIMBOT(new SimBotConfig());

  private final List<String> macAddresses;
  private final RobotConfig robotConfig;

  private RobotId(RobotConfig robotConfig, String... macAddresses) {
    this.macAddresses = List.of(macAddresses);
    this.robotConfig = robotConfig;
  }

  @Override
  public List<String> getMacAddresses() {
    return macAddresses;
  }

  public RobotConfig getRobotConfig() {
    return this.robotConfig;
  }
}
