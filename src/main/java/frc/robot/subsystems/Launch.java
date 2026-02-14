package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Launch extends SubsystemBase {
  private final SparkFlex motor1 =
      new SparkFlex(Constants.Launch.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex motor2 =
      new SparkFlex(Constants.Launch.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);

  SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          // Launch motors are 1:1 with fly wheel
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFollowers(Pair.of(motor2, true));

  // Vendor motor controller object
  // SparkMax motor1 = new SparkMax(31, MotorType.kBrushless);
  // SparkMax motor2 = new SparkMax(37, MotorType.kBrushless);
  // Create our SmartMotorController from our Spark and config with the NEO.
  SmartMotorController launchSMC = new SparkWrapper(motor1, DCMotor.getNEO(1), smcConfig);

  FlyWheelConfig shooterConfig =
      new FlyWheelConfig(launchSMC)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(6000))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {
    shooter.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.run(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
