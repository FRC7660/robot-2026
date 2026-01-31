package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// initializes index and funnel motors
public class Index extends SubsystemBase {
  private final SparkMax indexMotor =
      new SparkMax(Constants.Index.INDEX_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkMax funnelMotor =
      new SparkMax(Constants.Index.FUNNEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  public Index() {
    SparkMaxConfig indexConfig = configureIndexMotor();
    SparkMaxConfig funnelConfig = configureFunnelMotor();

    indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    funnelMotor.configure(
        funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Index Motor Config
  public SparkMaxConfig configureIndexMotor() {
    SparkMaxConfig indexConfig = new SparkMaxConfig();
    indexConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    return indexConfig;
  }

  // Funnel Motor Config
  public SparkMaxConfig configureFunnelMotor() {
    SparkMaxConfig funnelConfig = new SparkMaxConfig();
    funnelConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    return funnelConfig;
  }

  public void setIndexSpeed(double speed) {
    this.indexMotor.set(speed);
  }

  public void setFunnelSpeed(double speed) {
    this.funnelMotor.set(speed);
  }
}
