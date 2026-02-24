package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeLaunch extends SubsystemBase {
  private final SparkMax upperMotor =
      new SparkMax(Constants.IntakeLaunchConstants.UPPER_MOTOR_ID, MotorType.kBrushless);
  private final TalonSRX lowerMotor = new TalonSRX(Constants.IntakeLaunchConstants.LOWER_MOTOR_ID);
  private final SparkClosedLoopController upperPidController = upperMotor.getClosedLoopController();
  private final Timer atSpeedTimer = new Timer();

  public IntakeLaunch() {
    SparkMaxConfig upperConfig = new SparkMaxConfig();
    upperConfig.idleMode(IdleMode.kCoast).inverted(true);
    upperConfig
        .closedLoop
        .p(Constants.IntakeLaunchConstants.UPPER_VELOCITY_KP)
        .i(Constants.IntakeLaunchConstants.UPPER_VELOCITY_KI)
        .d(Constants.IntakeLaunchConstants.UPPER_VELOCITY_KD);
    upperMotor.configure(
        upperConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    lowerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public Command runIntake() {
    return Commands.run(
            () -> {
              setUpperDutyCycle(Constants.IntakeLaunchConstants.INTAKE_UPPER_DUTY_CYCLE);
              setLowerDutyCycle(Constants.IntakeLaunchConstants.INTAKE_LOWER_DUTY_CYCLE);
            },
            this)
        .finallyDo(this::stopAllImmediate)
        .withName("IntakeLaunchIntake");
  }

  public Command runOuttake() {
    return Commands.run(
            () -> {
              setUpperDutyCycle(Constants.IntakeLaunchConstants.OUTTAKE_UPPER_DUTY_CYCLE);
              setLowerDutyCycle(Constants.IntakeLaunchConstants.OUTTAKE_LOWER_DUTY_CYCLE);
            },
            this)
        .finallyDo(this::stopAllImmediate)
        .withName("IntakeLaunchOuttake");
  }

  public Command runUnstick() {
    return Commands.run(
            () -> {
              setUpperDutyCycle(Constants.IntakeLaunchConstants.UNSTICK_UPPER_DUTY_CYCLE);
              setLowerDutyCycle(Constants.IntakeLaunchConstants.UNSTICK_LOWER_DUTY_CYCLE);
            },
            this)
        .finallyDo(this::stopAllImmediate)
        .withName("IntakeLaunchUnstick");
  }

  public Command shootDefault() {
    return shoot(Constants.IntakeLaunchConstants.DEFAULT_LAUNCH_SPEED_RPM);
  }

  public Command shoot(double targetRpm) {
    return Commands.startRun(
            this::resetSpeedGate,
            () -> {
              setUpperVelocityRpm(targetRpm);
              if (isUpperAtSpeed(targetRpm)) {
                setLowerDutyCycle(Constants.IntakeLaunchConstants.LOWER_FEED_DUTY_CYCLE);
              } else {
                setLowerDutyCycle(0.0);
              }
            },
            this)
        .withName("IntakeLaunchShoot");
  }

  public Command stopShooting() {
    return Commands.runOnce(() -> setLowerDutyCycle(0.0), this)
        .andThen(Commands.waitSeconds(Constants.IntakeLaunchConstants.UPPER_STOP_DELAY_SECONDS))
        .andThen(Commands.runOnce(this::stopUpper, this))
        .withName("IntakeLaunchStop");
  }

  private void setUpperDutyCycle(double dutyCycle) {
    upperMotor.set(MathUtil.clamp(dutyCycle, -1.0, 1.0));
  }

  private void setLowerDutyCycle(double dutyCycle) {
    lowerMotor.set(ControlMode.PercentOutput, MathUtil.clamp(dutyCycle, -1.0, 1.0));
  }

  private void setUpperVelocityRpm(double targetRpm) {
    upperPidController.setSetpoint(-Math.abs(targetRpm), ControlType.kVelocity);
  }

  private boolean isUpperAtSpeed(double targetRpm) {
    double actualRpm = Math.abs(upperMotor.getEncoder().getVelocity());
    double errorRpm = Math.abs(Math.abs(targetRpm) - actualRpm);

    if (errorRpm <= Constants.IntakeLaunchConstants.UPPER_READY_TOLERANCE_RPM) {
      if (!atSpeedTimer.isRunning()) {
        atSpeedTimer.restart();
      }
    } else {
      resetSpeedGate();
    }

    return atSpeedTimer.hasElapsed(Constants.IntakeLaunchConstants.UPPER_READY_TIME_SECONDS);
  }

  private void stopUpper() {
    upperMotor.set(0.0);
    resetSpeedGate();
  }

  private void stopAllImmediate() {
    setLowerDutyCycle(0.0);
    stopUpper();
  }

  private void resetSpeedGate() {
    atSpeedTimer.stop();
    atSpeedTimer.reset();
  }
}
