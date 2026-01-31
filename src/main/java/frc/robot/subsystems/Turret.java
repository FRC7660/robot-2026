package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Turret {
  private final SparkMax turretMotor =
      new SparkMax(Constants.Turret.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  public Turret() {
    SparkMaxConfig turretConfig = configureTurretMotor();
    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Turret Motor Config
  public SparkMaxConfig configureTurretMotor() {
    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig.idleMode(IdleMode.kBrake).inverted(false);
    return turretConfig;
  }
}
