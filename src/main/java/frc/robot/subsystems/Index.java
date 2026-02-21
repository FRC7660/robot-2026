package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

// initializes index and funnel motors
public class Index extends SubsystemBase {
  private final SparkFlex indexMotor =
      new SparkFlex(Constants.Index.INDEX_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex funnelMotor =
      new SparkFlex(Constants.Index.FUNNEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  public Index() {
    SparkFlexConfig indexConfig = configureIndexMotor();
    SparkFlexConfig funnelConfig = configureFunnelMotor();

    indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    funnelMotor.configure(
        funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Index Motor Config
  public SparkFlexConfig configureIndexMotor() {
    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
    return indexConfig;
  }

  // Funnel Motor Config
  public SparkFlexConfig configureFunnelMotor() {
    SparkFlexConfig funnelConfig = new SparkFlexConfig();
    funnelConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(true);
    return funnelConfig;
  }

  public void setIndexSpeed(double speed) {
    this.indexMotor.set(speed);
  }

  public void setFunnelSpeed(double speed) {
    this.funnelMotor.set(speed);
  }

  private void run() {
    setIndexSpeed(Constants.Index.INDEX_SPEED);
    setFunnelSpeed(Constants.Index.FUNNEL_SPEED);
  }

  private void stop() {
    setIndexSpeed(0);
    setFunnelSpeed(0);
  }

  // public Command runCommand() {
  //   return startEnd(() -> run(), () -> stop());
  // }

  public Command setSystemSpeed(Double indexSpeed, Double funnelSpeed) {
    return runEnd(
        () -> {
          setIndexSpeed(indexSpeed);
          setFunnelSpeed(funnelSpeed);
        },
        () -> stop());
  }
}
