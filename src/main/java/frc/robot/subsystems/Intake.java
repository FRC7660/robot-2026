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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

// initializes intake arm and roller motors
public class Intake extends SubsystemBase {
  private final TalonFX liftMotor = new TalonFX(Constants.Intake.LIFT_MOTOR_ID);
  private final TalonFX rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

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
          .withClosedLoopController(175.0, 0.0, 20.0, RPM.of(1000), RPM.per(Second).of(6000))
          .withFeedforward(new ArmFeedforward(0.02, 0.005, 0.0))
          // sim
          .withSimClosedLoopController(175.0, 0, 20.0, RPM.of(1000), RPM.per(Second).of(6000))
          .withSimFeedforward(new ArmFeedforward(0.02, 0.005, 0.0))
          .withTelemetry("LiftMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 3.333)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.05))
          .withOpenLoopRampRate(Seconds.of(0.05));

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController liftSmartMotorController =
      new TalonFXWrapper(liftMotor, DCMotor.getKrakenX60(1), liftConfig);

  private ArmConfig liftCfg =
      new ArmConfig(liftSmartMotorController)
          .withSoftLimits(Degrees.of(-25), Degrees.of(110))
          .withHardLimit(Degrees.of(-30), Degrees.of(110))
          .withStartingPosition(Degrees.of(110))
          .withLength(Inches.of(8))
          .withMass(Pounds.of(8.466))
          .withTelemetry("Arm", TelemetryVerbosity.HIGH)
          .withHorizontalZero(Degrees.of(0.0));

  // Arm Mechanism
  private Arm lift = new Arm(liftCfg);

  public Intake() {
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // Update telemetry for the arm mechanism
    lift.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Update simulation physics and visualization
    lift.simIterate();
  }

  public Command setAngle(Double angle) {
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    return lift.setAngle(convertedAngle);
  }

  public Command setAngleAndStop(Double angle) {
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    return lift.runTo(convertedAngle, Angle.ofRelativeUnits(2, Degrees));
  }

  public void setAngleSetpoint(Double angle) {
    Angle convertedAngle = Angle.ofRelativeUnits(angle, Degrees);
    lift.setMechanismPositionSetpoint(convertedAngle);
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
    lift.setMechanismPositionSetpoint(Angle.ofRelativeUnits(-25.0, Degrees));
  }

  public Command fullIntake() {
    return run(
        () -> {
          setRollerSpeed(0.99);
          fullDeploy();
        });
  }
}
