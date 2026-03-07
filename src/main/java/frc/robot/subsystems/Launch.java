package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers;
import java.util.function.Supplier;
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
      new SparkFlex(Constants.LaunchConstants.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex motor2 =
      new SparkFlex(Constants.LaunchConstants.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);

  SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(0, 0, 0, RPM.of(6700), RPM.per(Second).of(6700 * 2))
          .withSimClosedLoopController(0, 0, 0, RPM.of(6700), RPM.per(Second).of(6700 * 2))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          .withSimFeedforward(new SimpleMotorFeedforward(0.115, 6.5, 3))
          // Telemetry name and verbosity level
          .withTelemetry("LaunchWheel", TelemetryVerbosity.HIGH)
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
          .withUpperSoftLimit(RPM.of(6700))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("YLaunchWheel", TelemetryVerbosity.HIGH);

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
   * Set the shooter velocity. Speed should be POSITIVE.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(double speed) {
    double absSpeed = -Math.abs(speed);
    AngularVelocity velocity = RPM.of(absSpeed);
    return shooter.run(velocity);
  }

  /** Calculate and return a velocity setpoint based on distance */
  public AngularVelocity getOptimalVelocity() {
    return RPM.of(15);
  }

  /**
   * Calculate and return a velocity setpoint based on distance to target (meters), interpolating
   * between configured table values.
   *
   * @param distanceMeters distance from robot to target in meters
   * @return interpolated launcher speed setpoint
   */
  public AngularVelocity getOptimalVelocity(double distanceMeters) {
    return RotationsPerSecond.of(interpolateSpeedRpm(distanceMeters));
  }

  /**
   * Calculate and return a velocity setpoint using turret's most recent target and pose data.
   *
   * @param turret Turret subsystem containing the latest pose and target data
   * @return interpolated launcher speed setpoint
   */
  public AngularVelocity getOptimalVelocity(Turret turret) {
    double baseDistance = getDistanceToTurretLastTargetMeters(turret);
    return getOptimalVelocity(adjustDistanceForAllianceZone(turret, baseDistance));
  }

  /**
   * Calculate the distance from the robot to the turret's most recently selected target.
   *
   * @param turret Turret subsystem containing the latest pose and target data
   * @return distance in meters
   */
  public double getDistanceToTurretLastTargetMeters(Turret turret) {
    Translation2d robotPosition = turret.getLastPose().getTranslation();
    Translation2d targetPosition = turret.getLastTarget();
    return robotPosition.getDistance(targetPosition);
  }

  private double interpolateSpeedRpm(double distanceMeters) {
    double[] distances = Constants.LaunchConstants.DISTANCE_TO_TARGET_METERS;
    double[] speeds = Constants.LaunchConstants.LAUNCH_SPEED_RPS;

    if (distances.length == 0 || speeds.length == 0 || distances.length != speeds.length) {
      throw new IllegalStateException(
          "Launch distance/speed tables must be non-empty and the same length.");
    }

    if (distanceMeters <= distances[0]) {
      return speeds[0];
    }

    int lastIndex = distances.length - 1;
    if (distanceMeters >= distances[lastIndex]) {
      return speeds[lastIndex];
    }

    for (int i = 0; i < lastIndex; i++) {
      double leftDistance = distances[i];
      double rightDistance = distances[i + 1];
      if (distanceMeters <= rightDistance) {
        double leftSpeed = speeds[i];
        double rightSpeed = speeds[i + 1];
        double t = (distanceMeters - leftDistance) / (rightDistance - leftDistance);
        return leftSpeed + t * (rightSpeed - leftSpeed);
      }
    }

    return speeds[lastIndex];
  }

  private double adjustDistanceForAllianceZone(Turret turret, double distanceMeters) {
    Translation2d robotPosition = turret.getLastPose().getTranslation();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    boolean inOwnAllianceZone =
        (alliance == DriverStation.Alliance.Red
                && robotPosition.getX() > TurretHelpers.RED_THRESHOLD_X)
            || (alliance == DriverStation.Alliance.Blue
                && robotPosition.getX() < TurretHelpers.BLUE_THRESHOLD_X);

    return inOwnAllianceZone ? distanceMeters + 1.0 : distanceMeters;
  }

  public void stop() {
    shooter.run(RPM.of(0));
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

  public Command shotSequenceStart(Index indexSystem) {
    Supplier<AngularVelocity> s_velSupplier = () -> getVelocity();
    Supplier<AngularVelocity> s_velSetpointSupplier = () -> getOptimalVelocity();
    Trigger optimalVelocityReached =
        // FIXME: the conversion of 60 here is very suspect
        new Trigger(
            () ->
                (s_velSupplier.get().in(RPM) >= (s_velSetpointSupplier.get().in(RPM) * 60) * 0.97));
    // *60 because units are broken
    // 3% margin of error
    return Commands.repeatingSequence(
            // Pause the funnel/index to allow the flywheel to re-spool
            Commands.runOnce(() -> indexSystem.setVelocitySetpointindex(RPM.of(0.0))),
            Commands.runOnce(() -> indexSystem.setVelocitySetpointfunnel(RPM.of(0.0))),
            Commands.runOnce(
                () -> {
                  // PLACEHOLDER: getOptimalVelocity should not just return 2000; attach the
                  // distance and SWM calculations
                  this.setVelocitySetpoint(s_velSetpointSupplier.get());
                }),
            Commands.waitUntil(() -> optimalVelocityReached.getAsBoolean()),
            // Resume funnel/index
            Commands.runOnce(() -> indexSystem.setVelocitySetpointfunnel(RPM.of(200.0))),
            Commands.runOnce(() -> indexSystem.setVelocitySetpointindex(RPM.of(120.0))),
            Commands.waitUntil(() -> optimalVelocityReached.negate().getAsBoolean()))
        .handleInterrupt(() -> shotSequenceEnd(indexSystem));
  }

  public Command shotSequenceStart(Index indexSystem, Turret turret) {
    Supplier<AngularVelocity> s_velSupplier = () -> getVelocity();
    Supplier<AngularVelocity> s_velSetpointSupplier = () -> getOptimalVelocity(turret);
    Trigger optimalVelocityReached =
        // FIXME: the conversion of 60 here is very suspect
        new Trigger(
            () ->
                (s_velSupplier.get().in(RPM) >= (s_velSetpointSupplier.get().in(RPM) * 60) * 0.97));
    // *60 because units are broken
    // 3% margin of error
    return Commands.repeatingSequence(
            // Pause the funnel/index to allow the flywheel to re-spool
            Commands.runOnce(() -> indexSystem.setVelocitySetpointindex(RPM.of(0.0))),
            Commands.runOnce(() -> indexSystem.setVelocitySetpointfunnel(RPM.of(0.0))),
            Commands.runOnce(() -> this.setVelocitySetpoint(s_velSetpointSupplier.get())),
            Commands.waitUntil(() -> optimalVelocityReached.getAsBoolean()),
            // Resume funnel/index
            Commands.runOnce(() -> indexSystem.setVelocitySetpointfunnel(RPM.of(200.0))),
            Commands.runOnce(() -> indexSystem.setVelocitySetpointindex(RPM.of(120.0))),
            Commands.waitUntil(() -> optimalVelocityReached.negate().getAsBoolean()))
        .handleInterrupt(() -> shotSequenceEnd(indexSystem));
  }

  private void shotSequenceEnd(Index indexSystem) {
    indexSystem.setVelocitySetpointindex(RPM.of(0.0));
    indexSystem.setVelocitySetpointfunnel(RPM.of(0.0));
    /**
     * This should set the voltage output of the flywheel to 0. Theoretically will bypass the
     * velocity setpoint due to the command being interrupted and just cause the launcher to spin
     * down.
     */
    shooter.setDutyCycleSetpoint(0);
  }
}
