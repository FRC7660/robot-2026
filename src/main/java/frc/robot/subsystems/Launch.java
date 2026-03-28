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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class Launch extends SubsystemBase {
  private static final InterpolatingDoubleTreeMap SPEED_TABLE = new InterpolatingDoubleTreeMap();

  static {
    SPEED_TABLE.put(Units.inchesToMeters(0), 35.0);
    SPEED_TABLE.put(Units.inchesToMeters(141), 44.0);
    SPEED_TABLE.put(Units.inchesToMeters(145), 45.0);
    SPEED_TABLE.put(Units.inchesToMeters(182), 50.0);
    SPEED_TABLE.put(Units.inchesToMeters(216), 55.0);
    SPEED_TABLE.put(Units.inchesToMeters(246), 60.0);
    SPEED_TABLE.put(Units.inchesToMeters(290), 65.0);
    SPEED_TABLE.put(Units.inchesToMeters(312), 80.0);
  }

  private final SparkFlex motor1 =
      new SparkFlex(Constants.LaunchConstants.MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);

  private final SparkFlex motor2 =
      new SparkFlex(Constants.LaunchConstants.MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);

  SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // ROBOT
          .withFeedforward(new SimpleMotorFeedforward(0.115, 0.108, 0.05))
          .withClosedLoopController(0.001, 0, 0, RPM.of(6700), RPM.per(Second).of(6700 * 2))
          // SIMULATION
          .withSimFeedforward(new SimpleMotorFeedforward(0.115, 0.108, 0.05))
          .withSimClosedLoopController(0.3, 0, 0, RPM.of(6700), RPM.per(Second).of(6700 * 2))
          // Telemetry name and verbosity level
          .withTelemetry("LaunchWheel", Constants.Telemetry.yamsVerbosity())
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
  SmartMotorController launchSMC = new SparkWrapper(motor1, DCMotor.getNeoVortex(1), smcConfig);

  FlyWheelConfig shooterConfig =
      new FlyWheelConfig(launchSMC)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(6700))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("YLaunchWheel", Constants.Telemetry.yamsVerbosity());

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  @AutoLogOutput(key = "Launch/Velocity")
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

  private void startVelocityClosedLoop() {
    shooter.getMotorController().startClosedLoopController();
  }

  private void setVelocityTargetNoRestart(AngularVelocity speed) {
    shooter.getMotorController().setVelocity(speed);
  }

  @AutoLogOutput(key = "Launch/SetpointRpm")
  public double getVelocitySetpointRpm() {
    return shooter.getMechanismSetpointVelocity().orElse(RPM.of(0)).in(RPM);
  }

  /**
   * Set the shooter velocity. Speed should be POSITIVE.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return Commands.startRun(
        this::startVelocityClosedLoop, () -> setVelocityTargetNoRestart(speed), this);
  }

  /** Calculate and return a velocity setpoint based on distance */
  @AutoLogOutput(key = "Launch/OptimalVelocityRpm")
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
    return RotationsPerSecond.of(SPEED_TABLE.get(distanceMeters));
  }

  /**
   * Calculate and return a velocity setpoint using turret's most recent target and pose data.
   *
   * @param turret Turret subsystem containing the latest pose and target data
   * @return interpolated launcher speed setpoint
   */
  public AngularVelocity getOptimalVelocity(Turret turret) {
    Pose2d pose = turret.getLastPose();
    Translation2d robotPosition = pose.getTranslation();
    Translation2d turretPosition =
        robotPosition.plus(Constants.Turret.POSITION_ROBOT_FRAME.rotateBy(pose.getRotation()));
    Translation2d targetPosition = turret.getLastTarget();
    if (robotPosition.equals(new Translation2d()) && targetPosition.equals(new Translation2d())) {
      return RotationsPerSecond.of(SPEED_TABLE.get(0.0));
    }
    double baseDistance = turretPosition.getDistance(targetPosition);
    double adjustedDistance = adjustDistanceForAllianceZone(robotPosition, baseDistance);
    SmartDashboard.putNumber("baseDistance", baseDistance);
    return getOptimalVelocity(adjustedDistance);
  }

  /**
   * Calculate the distance from the robot to the turret's most recently selected target.
   *
   * @param turret Turret subsystem containing the latest pose and target data
   * @return distance in meters
   */
  public double getDistanceToTurretLastTargetMeters(Turret turret) {
    Pose2d pose = turret.getLastPose();
    Translation2d turretPosition =
        pose.getTranslation()
            .plus(Constants.Turret.POSITION_ROBOT_FRAME.rotateBy(pose.getRotation()));
    return turretPosition.getDistance(turret.getLastTarget());
  }

  private double adjustDistanceForAllianceZone(Translation2d robotPosition, double distanceMeters) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    boolean inOwnAllianceZone =
        (alliance == DriverStation.Alliance.Red
                && robotPosition.getX() > TurretHelpers.RED_THRESHOLD_X)
            || (alliance == DriverStation.Alliance.Blue
                && robotPosition.getX() < TurretHelpers.BLUE_THRESHOLD_X);

    return inOwnAllianceZone ? distanceMeters + 0.75 : distanceMeters;
  }

  private boolean isAimed(Turret turret) {
    double currentSetpoint = turret.getSetpointDegrees();
    double currentAngle = turret.getCurrentAngleDegrees();
    return Math.abs(currentSetpoint - currentAngle) < 15;
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

  public Command shotSequenceStartWithTurret(Index indexSystem, Turret turret) {
    return Commands.parallel(turret.autoSetAngle(), shotSequenceStart(indexSystem, turret))
        .handleInterrupt(turret::freeze);
  }

  public Command shotSequenceStart(Index indexSystem, Turret turret) {
    Supplier<AngularVelocity> s_velSupplier = () -> getVelocity();
    Supplier<AngularVelocity> s_velSetpointSupplier = () -> getOptimalVelocity(turret);
    Trigger optimalVelocityReached =
        new Trigger(() -> isAtSpeed(s_velSupplier.get(), s_velSetpointSupplier.get()));
    Trigger lowVelocityHit =
        new Trigger(() -> isLowVelocity(s_velSupplier.get(), s_velSetpointSupplier.get()));
    return Commands.parallel(
            Commands.startRun(
                this::startVelocityClosedLoop,
                () -> setVelocityTargetNoRestart(s_velSetpointSupplier.get()),
                this),
            Commands.repeatingSequence(
                // Pause funnel/index until shooter reaches target speed.
                Commands.runOnce(() -> indexSystem.setVelocitySetpointindex(RPM.of(0.0))),
                Commands.runOnce(() -> indexSystem.setVelocitySetpointfunnel(RPM.of(0.0))),
                Commands.waitUntil(
                    () -> {
                      return (optimalVelocityReached.getAsBoolean() && isAimed(turret));
                    }),
                // Feed while shooter remains at speed.
                Commands.runOnce(
                    () ->
                        indexSystem.setVelocitySetpointfunnel(
                            RPM.of(Constants.LaunchConstants.FUNNEL_RPM))),
                Commands.runOnce(
                    () ->
                        indexSystem.setVelocitySetpointindex(
                            RPM.of(Constants.LaunchConstants.INDEX_RPM))),
                Commands.waitUntil(
                    () -> {
                      return (lowVelocityHit.getAsBoolean() || !isAimed(turret));
                    })))
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

  private static boolean isAtSpeed(AngularVelocity actualVelocity, AngularVelocity targetVelocity) {
    double actualRps = Math.abs(actualVelocity.in(RotationsPerSecond));
    double targetRps = Math.abs(targetVelocity.in(RotationsPerSecond));
    return actualRps >= targetRps * 0.90;
  }

  private static boolean isLowVelocity(
      AngularVelocity actualVelocity, AngularVelocity targetVelocity) {
    double actualRps = Math.abs(actualVelocity.in(RotationsPerSecond));
    double targetRps = Math.abs(targetVelocity.in(RotationsPerSecond));
    return actualRps <= targetRps * 0.60;
  }
}
