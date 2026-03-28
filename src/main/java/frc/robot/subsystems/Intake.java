package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.DashboardTelemetry;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

// initializes intake arm and roller motors
public class Intake extends SubsystemBase {
  private final TalonFX liftMotor = new TalonFX(Constants.Intake.LIFT_MOTOR_ID);
  private final TalonFX rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

  // Limit switch for intake
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Intake.LIMIT_SWITCH_PORT);

  // Roller Motor Config
  public void configureRollerMotor() {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0);

  private SmartMotorControllerConfig liftConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              170.0, 0.0, 15.0, RPM.of(50), RPM.per(Second).of(300)) // decreased these for safety
          .withFeedforward(new ArmFeedforward(0.02, 0.005, 4.5))
          // sim
          .withSimClosedLoopController(175.0, 0, 20.0, RPM.of(1000), RPM.per(Second).of(6000))
          .withSimFeedforward(new ArmFeedforward(0.02, 0.005, 0.0))
          .withTelemetry("IntakeArm", Constants.Telemetry.yamsVerbosity())
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 3.333)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.05))
          .withOpenLoopRampRate(Seconds.of(0.05));

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController liftSmartMotorController =
      new TalonFXWrapper(liftMotor, DCMotor.getKrakenX60(1), liftConfig);

  private ArmConfig liftCfg =
      new ArmConfig(liftSmartMotorController)
          .withSoftLimits(Degrees.of(-30), Degrees.of(115))
          .withHardLimit(Degrees.of(-35), Degrees.of(115))
          .withStartingPosition(Degrees.of(113))
          .withLength(Inches.of(8))
          .withMass(Pounds.of(8.466))
          .withTelemetry("YIntakeArm", Constants.Telemetry.yamsVerbosity())
          .withHorizontalZero(Degrees.of(0.0));

  // Arm Mechanism
  private Arm lift = new Arm(liftCfg);
  private Angle armAngleOffset = Degrees.of(0.0);
  private double lastRequestedAngle = 115.0;

  public Intake() {
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // Update telemetry for the arm mechanism
    lift.updateTelemetry();

    // Update limit switch status on SmartDashboard
    DashboardTelemetry.putBoolean("Intake/LimitSwitchPressed", isLimitSwitchPressed());
  }

  @Override
  public void simulationPeriodic() {
    // Update simulation physics and visualization
    lift.simIterate();
  }

  public Command setAngle(Double angle) {
    lastRequestedAngle = angle;
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    return lift.setAngle(convertedAngle);
  }

  @AutoLogOutput(key = "Intake/ArmAngle")
  public Angle getAngle() {
    return lift.getAngle();
  }

  @AutoLogOutput(key = "Intake/ArmSetpointDegrees")
  public double getArmSetpointDegrees() {
    return liftSmartMotorController
        .getMechanismPositionSetpoint()
        .map(angle -> angle.in(Degrees))
        .orElse(0.0);
  }

  public Command setAngleAndStop(Double angle) {
    lastRequestedAngle = angle;
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    return lift.runTo(convertedAngle, Angle.ofRelativeUnits(2, Degrees));
  }

  public void setAngleSetpoint(Double angle) {
    lastRequestedAngle = angle;
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    lift.setMechanismPositionSetpoint(convertedAngle);
    setMotorBrake(true); // Re-enable brake mode when setpoint is changed
  }

  public Command set(double dutycycle) {
    return lift.set(dutycycle);
  }

  public Command sysId() {
    return lift.sysId(Volts.of(7), Volts.of(2).per(Seconds), Seconds.of(4));
  }

  public void setArmSpeed(double speed) {}

  public void setRollerSpeed(double speed) {
    this.rollerMotor.setControl(rollerDutyCycle.withOutput(speed));
  }

  public Supplier<Double> getRollerSpeed() {
    return (() -> this.rollerMotor.get());
  }

  @AutoLogOutput(key = "Intake/RollerOutput")
  public double getRollerOutput() {
    return rollerMotor.get();
  }

  @AutoLogOutput(key = "Intake/RollerVelocityRps")
  public double getRollerVelocityRps() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "Intake/LiftSupplyCurrentAmps")
  public double getLiftSupplyCurrentAmps() {
    return liftMotor.getSupplyCurrent().getValueAsDouble();
  }

  private void runRoller() {
    setRollerSpeed(Constants.Intake.ROLLER_SPEED);
  }

  public void stopRoller() {
    setRollerSpeed(0);
  }

  public Command activateRoller() {
    return startEnd(() -> runRoller(), () -> stopRoller());
  }

  public Command runCommand(DoubleSupplier speedSupplier) {
    return runEnd(
        () -> {
          double speed = speedSupplier.getAsDouble();
          setRollerSpeed(speed);
        },
        () -> stopRoller());
  }

  public void fullDeploy() {
    setAngleSetpoint(-30.0);
  }

  @AutoLogOutput(key = "Intake/ArmAngleOffsetDegrees")
  public double getArmAngleOffsetDegrees() {
    return armAngleOffset.in(Degrees);
  }

  public void adjustArmAngleOffset(Angle deltaAngle) {
    armAngleOffset = armAngleOffset.plus(deltaAngle);
  }

  public void adjustArmAngleOffsetAndReapplySetpoint(Angle deltaAngle) {
    armAngleOffset = armAngleOffset.plus(deltaAngle);
    Angle currentPos = liftSmartMotorController.getMechanismPosition();
    liftSmartMotorController.setPosition(currentPos.minus(deltaAngle));
    setAngleSetpoint(lastRequestedAngle);
  }

  public void incrementArmAngleOffsetDegrees() {
    adjustArmAngleOffset(Degrees.of(1.0));
  }

  public void decrementArmAngleOffsetDegrees() {
    adjustArmAngleOffset(Degrees.of(-1.0));
  }

  public Command toggleIntake() {
    return Commands.runOnce(
        () -> {
          Angle setpointAngle = lift.getMechanismSetpoint().orElse(getAngle());
          if (setpointAngle.in(Degrees) < 0) {
            setAngleSetpoint(70.0);
          } else {
            setAngleSetpoint(-30.0);
          }
        });
  }

  public Command fullIntake() {
    return run(
        () -> {
          setRollerSpeed(0.99);
          fullDeploy();
        });
  }

  public Command retract() {
    return run(
        () -> {
          setRollerSpeed(0.0);
          setAngleSetpoint(115.0);
        });
  }

  /**
   * Returns true if the limit switch is pressed, false otherwise.
   *
   * @return true if the limit switch is pushed, false if not
   */
  @AutoLogOutput(key = "Intake/LimitSwitchPressed")
  public boolean isLimitSwitchPressed() {
    return limitSwitch.get();
  }

  public Command zeroArm() {
    return run(() -> {
          // Run the lift motor in positive direction, bypassing soft limits
          liftMotor.set(0.2); // 20% power in positive direction
        })
        .until(
            () ->
                (isLimitSwitchPressed()
                    ||
                    // Stop if limit switch pressed or current exceeds 25A
                    liftMotor.getSupplyCurrent().getValueAsDouble() > 25.0))
        .andThen(
            () -> {
              // Stop the motor
              liftMotor.set(0);
              // Reset the arm position to 115 degrees at the hard stop
              liftSmartMotorController.setPosition(Degrees.of(115.0));
              armAngleOffset = Degrees.of(0.0);
              // Set the arm position setpoint to 115 degrees after calibration
              setAngleSetpoint(115.0);
            })
        .handleInterrupt(
            () -> {
              liftMotor.set(0.0);
            });
  }

  public void setMotorBrake(boolean brake) {
    if (brake == true) {
      lift.getMotor().setIdleMode(MotorMode.BRAKE);
    } else {
      lift.getMotor().setIdleMode(MotorMode.COAST);
    }
  }
}
