package frc.robot.subsystems.generic.roller;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.util.CanID;

public class RollerIODoubleTalonFX extends RollerIOTalonFX {
  public static class RollerConstantsDoubleTalonFX extends RollerConstantsTalonFX {
    public final CanID followerCanId;
    public MotorAlignmentValue followerAllignment = MotorAlignmentValue.Opposed;

    public RollerConstantsDoubleTalonFX(CanID mainMotor, CanID followerMotor) {
      super(mainMotor);
      this.followerCanId = followerMotor;
    }
  }

  private final TalonFX follower;

  public RollerIODoubleTalonFX(RollerConstantsDoubleTalonFX constants) {
    super(constants);

    follower = constants.followerCanId.getTalon();

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = constants.neutralMode;
    config.CurrentLimits.withSupplyCurrentLimit(constants.currentLimit);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> follower.getConfigurator().apply(config));
    tryUntilOk(
        5,
        () ->
            follower.setControl(new Follower(roller.getDeviceID(), constants.followerAllignment)));
  }
}
