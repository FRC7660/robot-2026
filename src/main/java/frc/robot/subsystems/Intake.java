package frc.robot.subsystems;

import com.revrobotics.spark.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /* ---------------- Hardware ---------------- */
  private final SparkMax liftMotor;
  private final SparkMax rollerMotor;
  //private final DigitalInput liftLimitSwitch;

  /* ---------------- Control ---------------- */
  private final PIDController liftPID;
  private double liftTargetAngle = 0.0; // in degrees
  private boolean homed = false;

  /* ---------------- Constructor ---------------- */
  public Intake() {
    liftMotor = new SparkMax(Constants.Intake.kLiftMotorId, MotorType.kBrushless);
    rollerMotor = new SparkMax(Constants.Intake.kRollerMotorId, MotorType.kBrushless);
    //liftLimitSwitch = new DigitalInput(Constants.Intake.kLiftLimitChannel);

    // -------- Lift Motor Config --------
    SparkMaxConfig liftConfig = new SparkMaxConfig();
    liftConfig.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(Constants.Intake.kLiftCurrentLimit)
              .inverted(false);
    liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------- Roller Motor Config --------
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.Intake.kRollerCurrentLimit)
                .inverted(false);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /* ---------------- Periodic ---------------- */
  @Override
  public void periodic() {
    // ---- Homing Logic ----
    if (!homed) {
      if (isLiftAtLimit()) {
        liftMotor.getEncoder().setPosition(0.0); // reset encoder at home
        liftMotor.set(0.0);
        homed = true;
      } else {
        liftMotor.set(-Constants.Intake.kHomingSpeed);
      }
      return; // skip PID control until homed
    }

    // ---- Normal Control ----
    double currentAngle = getArmAngle(); // in degrees
    double output = liftPID.calculate(currentAngle, liftTargetAngle);
    liftMotor.set(output);

    // ---- Telemetry ----
    SmartDashboard.putNumber("Lift Arm Angle", currentAngle);
    SmartDashboard.putNumber("Lift PID Output", output);
  }

  /* ---------------- Lift Commands ---------------- */
  /** Move intake down to ground pickup */
  public void deploy() {
    liftTargetAngle = Constants.Intake.kDeployedAngle; // in degrees
  }

  /** Move intake up into frame */
  public void stow() {
    liftTargetAngle = Constants.Intake.kStowedAngle; // in degrees
  }

  /** Call once on robot init or disabled */
  public void home() {
    homed = false;
  }

  public boolean isHomed() {
    return homed;
  }

//  private boolean isLiftAtLimit() {
//  return !liftLimitSwitch.get(); // depends on wiring
//}

  /** Converts motor encoder units to physical arm angle in degrees */
  public double getArmAngle() {
    double encoderUnits = liftMotor.getEncoder().getPosition();
    return 3.6 * encoderUnits; // 3.6° per encoder unit (10 counts per motor rotation, 10:1 gear ratio)
  }

  /* ---------------- Roller Commands ---------------- */
  /** Pull game piece in */
  public void intake() {
    rollerMotor.set(Constants.Intake.kIntakeSpeed);
  }

  /** Push game piece out */
  public void eject() {
    rollerMotor.set(-Constants.Intake.kEjectSpeed);
  }

  public void stopRoller() {
    rollerMotor.set(0.0);
  }

  /* ---------------- Safety ---------------- */
  public void stopAll() {
    liftMotor.set(0.0);
    rollerMotor.set(0.0);
  }
}