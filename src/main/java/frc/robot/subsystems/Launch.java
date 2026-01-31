package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Launch {
  private final SparkMax motor1 =
      new SparkMax(Constants.Launch.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkMax motor2 =
      new SparkMax(Constants.Launch.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);

  public Launch() {
    SparkMaxConfig baseConfig = new SparkMaxConfig();
    baseConfig.idleMode(IdleMode.kCoast);

    motor1.configure(
        new SparkMaxConfig().apply(baseConfig).inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    motor2.configure(
        new SparkMaxConfig().apply(baseConfig).inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    this.motor1.set(speed);
    this.motor2.set(speed);
  }
}
