package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

// initializes intake arm and roller motors
public class Intake {
  private final SparkMax liftMotor =
      new SparkMax(Constants.Intake.LIFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkMax rollerMotor =
      new SparkMax(Constants.Intake.ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  public Intake() {
    // Configure motors with settings from Constants
    SparkMaxConfig liftConfig = configureLiftMotor();
    SparkMaxConfig rollerConfig = configureRollerMotor();

    liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setArmSpeed(double speed) {
    this.liftMotor.set(speed);
  }

  public void setRollerSpeed(double speed) {
    this.rollerMotor.set(speed);
  }

  // Lift Motor Config
  public SparkMaxConfig configureLiftMotor() {
    SparkMaxConfig liftConfig = new SparkMaxConfig();
    liftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Intake.LIFT_CURRENT_LIMIT)
        .inverted(false);
    return liftConfig;
  }

  // Roller Motor Config
  public SparkMaxConfig configureRollerMotor() {
    SparkMaxConfig liftConfig = new SparkMaxConfig();
    liftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Intake.ROLLER_CURRENT_LIMIT)
        .inverted(false);
    return liftConfig;
  }
}
