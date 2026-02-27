package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeLaunch extends SubsystemBase {
  private final SparkMax upperMotor =
      new SparkMax(Constants.IntakeLaunchConstants.UPPER_MOTOR_ID, MotorType.kBrushless);
  private final TalonSRX lowerMotor = new TalonSRX(Constants.IntakeLaunchConstants.LOWER_MOTOR_ID);
  private final SmartMotorControllerConfig upperMotorConfig =
      new SmartMotorControllerConfig(this)
          .withTelemetry("upperConfig", TelemetryVerbosity.HIGH)
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(true)
          .withGearing(new MechanismGearing(1))
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopController(
              Constants.IntakeLaunchConstants.UPPER_VELOCITY_KP,
              Constants.IntakeLaunchConstants.UPPER_VELOCITY_KI,
              Constants.IntakeLaunchConstants.UPPER_VELOCITY_KD);
  private final SmartMotorController upperMotorController =
      new SparkWrapper(upperMotor, DCMotor.getNEO(1), upperMotorConfig);
  private final FlyWheelConfig upperFlyWheelConfig =
      new FlyWheelConfig(upperMotorController)
         .withTelemetry("upperFlyWheel", TelemetryVerbosity.HIGH)
         .withMOI(KilogramSquareMeters.of(0.02));
  private final FlyWheel upperFlyWheel = new FlyWheel(upperFlyWheelConfig);
  private final Timer atSpeedTimer = new Timer();

  public IntakeLaunch() {
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
    upperFlyWheel.setDutyCycleSetpoint(MathUtil.clamp(dutyCycle, -1.0, 1.0));
  }

  private void setLowerDutyCycle(double dutyCycle) {
    lowerMotor.set(ControlMode.PercentOutput, MathUtil.clamp(dutyCycle, -1.0, 1.0));
  }

  private void setUpperVelocityRpm(double targetRpm) {
    upperFlyWheel.setMechanismVelocitySetpoint(RPM.of(targetRpm));
  }

  private boolean isUpperAtSpeed(double targetRpm) {
    double actualRpm = Math.abs(upperFlyWheel.getSpeed().in(RPM));
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
    upperFlyWheel.setDutyCycleSetpoint(0.0);
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

  @Override
  public void periodic() {
    upperFlyWheel.updateTelemetry();
    SmartDashboard.putNumber("Launch Speed", upperFlyWheel.getSpeed().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    upperFlyWheel.simIterate();
  }
}
