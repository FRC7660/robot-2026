package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class Launch {
  private final SparkFlex motor1 =
      new SparkFlex(Constants.Launch.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex motor2 =
      new SparkFlex(Constants.Launch.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);

  public Launch() {
    SparkFlexConfig baseConfig = new SparkFlexConfig();
    baseConfig.idleMode(IdleMode.kCoast);

    motor1.configure(
        new SparkFlexConfig().apply(baseConfig).inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    motor2.configure(
        new SparkFlexConfig().apply(baseConfig).inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    this.motor1.set(speed);
    this.motor2.set(speed);
  }
}
