package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

// initializes intake arm and roller motors
public class Intake extends SubsystemBase {
  private final TalonFX liftMotor = new TalonFX(Constants.Intake.LIFT_MOTOR_ID);
  private final TalonFX rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

  private final DutyCycleOut liftDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0);

  public Intake() {
    configureLiftMotor();
    configureRollerMotor();
  }

  public void setArmSpeed(double speed) {
    this.liftMotor.setControl(liftDutyCycle.withOutput(speed));
  }

  public void setRollerSpeed(double speed) {
    this.rollerMotor.setControl(rollerDutyCycle.withOutput(speed));
  }

  // Lift Motor Config
  public void configureLiftMotor() {
    TalonFXConfiguration liftConfig = new TalonFXConfiguration();
    liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    liftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    liftMotor.getConfigurator().apply(liftConfig);
  }

  // Roller Motor Config
  public void configureRollerMotor() {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }
  
  private void run() {
    setRollerSpeed(Constants.Intake.ROLLER_SPEED);
  }

  private void stop() {
    setRollerSpeed(0);
   }
  
  public Command runCommand() {
    return startEnd(() -> run(), () -> stop());
  }
  
   public Command runCommand(DoubleSupplier speedSupplier) {
    return runEnd(
        () -> {
          double speed = speedSupplier.getAsDouble();
          setRollerSpeed(speed);
        },
        () -> stop());
  }

  


}
