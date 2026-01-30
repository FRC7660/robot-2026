package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

// initializes intake arm and roller motors
public class Intake {
    private final SparkMax liftMotor = 
        new SparkMax(Constants.IntakeConstants.LIFT_MOTOR_ID, Constants.IntakeConstants.INTAKE_TYPE);
    //Lift ID = 15
    private final SparkMax rollerMotor =
        new SparkMax(Constants.IntakeConstants.ROLLER_MOTOR_ID, Constants.IntakeConstants.INTAKE_TYPE);
    //Motor ID = 17
    public void setArmSpeed(double speed) {
        this.liftMotor.set(speed);
    }

    public void setRollerSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    //Lift Motor Config 
    public SparkMaxConfig configureLiftMotor() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.idleMode(IdleMode.kBrake)
                  .smartCurrentLimit(Constants.IntakeConstants.LIFT_CURRENT_LIMIT)
                  .inverted(false);
        return liftConfig; 
    }
    // Roller Motor Config
    public SparkMaxConfig configureRollerMotor() {
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        liftConfig.idleMode(IdleMode.kBrake)
                  .smartCurrentLimit(Constants.IntakeConstants.ROLLER_CURRENT_LIMIT)
                  .inverted(false);
        return liftConfig; 
}
}