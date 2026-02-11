package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
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
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          // Sim feedforward: kS for static friction, kG for gravity, kV for velocity
          // Values tuned for 3ft, 1lb arm with 12:1 gearing
          .withSimFeedforward(new ArmFeedforward(0.1, 0.8, 1.5))
          // Telemetry name and verbosity level
          .withTelemetry("LiftMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController liftSmartMotorController =
      new TalonFXWrapper(liftMotor, DCMotor.getKrakenX60(1), liftConfig);

  private ArmConfig liftCfg =
      new ArmConfig(liftSmartMotorController)
          // Soft limit is applied to the SmartMotorControllers PID
          .withSoftLimits(Degrees.of(-20), Degrees.of(40))
          // Hard limit is applied to the simulation.
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          // Starting position is where your arm starts
          .withStartingPosition(Degrees.of(-5))
          // Length and mass of your arm for sim.
          .withLength(Feet.of(3))
          .withMass(Pounds.of(1))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("Arm", TelemetryVerbosity.HIGH);

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
    return lift.run(convertedAngle);
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

  private void stopRoller() {
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
}
