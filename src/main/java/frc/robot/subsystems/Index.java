package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
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

// initializes index and funnel motors
public class Index extends SubsystemBase {
  private final SparkFlex indexMotor =
      new SparkFlex(Constants.Index.INDEX_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex funnelMotor =
      new SparkFlex(Constants.Index.FUNNEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  SmartMotorControllerConfig indexConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(0, 0, 0, RPM.of(1000), RPM.per(Second).of(4000))
          .withSimClosedLoopController(0, 0, 0, RPM.of(1000), RPM.per(Second).of(4000))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          .withSimFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          // Telemetry name and verbosity level
          .withTelemetry("indexerMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to
          // the gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  // SparkMax spark1 = new SparkMax(40, SparkLowLevel.MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  SmartMotorController sparkSmartMotorController1 =
      new SparkWrapper(indexMotor, DCMotor.getNEO(1), indexConfig);

  FlyWheelConfig indexingConfig =
      new FlyWheelConfig(sparkSmartMotorController1)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(.5))
          // Maximum speed of the index.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("indexer", TelemetryVerbosity.HIGH);

  // index Mechanism
  private FlyWheel indexer = new FlyWheel(indexingConfig);

  /**
   * Gets the current velocity of the index.
   *
   * @return index velocity.
   */
  public AngularVelocity getVelocityindex() {
    return indexer.getSpeed();
  }

  /**
   * Set the index velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpointindex(AngularVelocity speed) {
    indexer.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the index velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocityindex(AngularVelocity speed) {
    return indexer.run(speed);
  }

  /**
   * Set the dutycycle of the index.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setindex(double dutyCycle) {
    return indexer.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexer.updateTelemetry();
    funneler.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    indexer.simIterate();
    funneler.simIterate();
  }

  // Funnel Motor Config
  SmartMotorControllerConfig funnelConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(0, 0, 0, RPM.of(1000), RPM.per(Second).of(4000))
          .withSimClosedLoopController(0, 0, 0, RPM.of(1000), RPM.per(Second).of(4000))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          .withSimFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to
          // the gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  // SparkMax spark2 = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  SmartMotorController sparkSmartMotorController2 =
      new SparkWrapper(funnelMotor, DCMotor.getNEO(1), funnelConfig);

  FlyWheelConfig funnelingConfig =
      new FlyWheelConfig(sparkSmartMotorController2)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the funnel.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("funnel", TelemetryVerbosity.HIGH);

  // Funnel Mechanism
  private FlyWheel funneler = new FlyWheel(funnelingConfig);

  /**
   * Gets the current velocity of the funnel.
   *
   * @return funnel velocity.
   */
  public AngularVelocity getVelocityfunnel() {
    return funneler.getSpeed();
  }

  /**
   * Set the funnel velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpointfunnel(AngularVelocity speed) {
    funneler.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the funnel velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocityfunnel(AngularVelocity speed) {
    return funneler.run(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setfunnel(double dutyCycle) {
    return funneler.set(dutyCycle);
  }
}
