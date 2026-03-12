package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.DashboardTelemetry;
import frc.robot.lib.TurretHelpers;
import frc.robot.lib.TurretZeroPoint;
import java.util.Optional;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/** Simple turret subsystem that holds a NEO Vortex and a supplier for the robot pose. */
public class Turret extends SubsystemBase {
  private final SparkFlex turretMotor =
      new SparkFlex(Constants.Turret.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final Supplier<Pose2d> getPose;
  // YAMS controller + mechanism
  private final SmartMotorController turretSmartMotorController;
  private final Pivot turretPivot;
  // Most recent setpoint sent to the mechanism controller (mechanism frame).
  private Rotation2d lastSetpoint = new Rotation2d();
  // Most recently chosen target (field coordinates, meters)
  private Translation2d lastTarget = new Translation2d();
  // Most recent pose used for targeting
  private Pose2d lastPose = new Pose2d();
  // Most recent field-relative angle to target
  private Rotation2d lastFieldAngle = new Rotation2d();
  // Most recent desired absolute turret angle (deg, after clamp)
  private double lastDesiredAbsDeg = 0.0;
  // Most recent practical rotation command (deg)
  private double lastRotationCommandDeg = 0.0;
  // Zero-point regulator
  private final TurretZeroPoint zeroPoint;

  /** Default constructor. Uses a trivial Pose2d supplier (origin) when no supplier is provided. */
  public Turret() {
    this(() -> new Pose2d());
  }

  /**
   * Construct a Turret with a pose supplier function.
   *
   * @param getPose a Supplier that returns the current {@link Pose2d} of the robot
   */
  public Turret(Supplier<Pose2d> getPose) {
    this.getPose = getPose;

    // Initialize the motor encoder so turret angle starts at 135 degrees.
    try {
      double turretRotations = 165.0 / 360.0;
      double motorRotations = turretRotations * Constants.Turret.TURRET_GEAR_RATIO;
    } catch (Throwable t) {
      // Ignore if encoder API unavailable in sim.
    }

    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withExponentialProfile(Volts.of(12), RPM.of(120), RPM.per(Second).of(400))
            // ROBOT
            .withClosedLoopController(9.0, 0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.0, 4.2, 0.0))
            // SIM
            .withSimClosedLoopController(10.0, 0, 0.2)
            .withSimFeedforward(new SimpleMotorFeedforward(0.0, 5.1, 0.0))
            // Telemetry name and verbosity level
            .withTelemetry("TurretTurret", Constants.Telemetry.yamsVerbosity())
            // Gearing from motor rotor to turret.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.05))
            .withOpenLoopRampRate(Seconds.of(0.05));

    turretSmartMotorController = new SparkWrapper(turretMotor, DCMotor.getNEO(1), smcConfig);

    PivotConfig pivotConfig =
        new PivotConfig(turretSmartMotorController)
            .withHardLimit(Degrees.of(-175), Degrees.of(175))
            .withSoftLimits(Degrees.of(-165), Degrees.of(165))
            .withStartingPosition(Degrees.of(0))
            .withTelemetry("YTurretTurret", Constants.Telemetry.yamsVerbosity())
            .withMOI(Meters.of(0.254), Pounds.of(2));

    turretPivot = new Pivot(pivotConfig);
    zeroPoint = new TurretZeroPoint();
  }

  /**
   * Set the turret desired angle relative to the robot forward direction. This is a stub that
   * records the setpoint; convert to motor commands in a follow-up change.
   *
   * @param robotRelativeAngle turret angle relative to robot forward
   */
  public void setTurretSetpoint(Rotation2d robotRelativeAngle) {
    // TODO: need to add "only move if greater than"
    double mechanismDeg = robotToMechanismDeg(robotRelativeAngle.getDegrees());
    Rotation2d mechanismAngle = Rotation2d.fromDegrees(mechanismDeg);
    turretSmartMotorController.setPosition(mechanismAngle.getMeasure());
    this.lastSetpoint = mechanismAngle;
  }

  /** Get the most recent turret setpoint (robot-relative angle). Primarily useful for testing. */
  public Rotation2d getLastSetpoint() {
    return lastSetpoint;
  }

  public Command autoSetAngle() {
    return turretPivot
        .setAngle(() -> getRobotRelativeAngle().getMeasure())
        .finallyDo(() -> freeze());
  }

  /**
   * Compute the desired turret angle relative to the robot using the stored pose supplier and
   * TurretHelpers decision logic.
   *
   * @return desired turret angle relative to the robot forward direction
   */
  public Rotation2d getRobotRelativeAngle() {
    Pose2d pose = getPose.get();
    this.lastPose = pose;
    Translation2d robotPos = pose.getTranslation();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Translation2d target = TurretHelpers.getTarget(robotPos, alliance);
    // record the chosen target for external inspection / dashboard
    this.lastTarget = target;
    Rotation2d fieldAngle = TurretHelpers.getFieldRelativeAngle(robotPos, target);
    this.lastFieldAngle = fieldAngle;
    Rotation2d robotRelative = TurretHelpers.getRobotRelative(fieldAngle, pose.getRotation());
    robotRelative = Rotation2d.fromDegrees(normalizeToSigned180(robotRelative.getDegrees()));
    return robotRelative;
  }

  public void freeze() {
    turretMotor.set(0);
  }

  /**
   * Returns the last computed target translation (field coordinates).
   *
   * @return last chosen {@link Translation2d} target
   */
  public Translation2d getLastTarget() {
    return lastTarget;
  }

  public Pose2d getLastPose() {
    return lastPose;
  }

  public Rotation2d getLastFieldAngle() {
    return lastFieldAngle;
  }

  public double getLastDesiredAbsDeg() {
    return lastDesiredAbsDeg;
  }

  public double getLastRotationCommandDeg() {
    return lastRotationCommandDeg;
  }

  @Override
  public void periodic() {
    turretPivot.updateTelemetry();
    Rotation2d calcRobotRelative = getRobotRelativeAngle();
    Optional<Angle> sp = turretSmartMotorController.getMechanismPositionSetpoint();
    double setPoint = sp.isPresent() ? normalizeToSigned180(sp.get().in(Degrees)) : -1.0;
    double currentDegSigned =
        normalizeToSigned180(turretSmartMotorController.getMechanismPosition().in(Degrees));
    double calcSetpointDegSigned = calcRobotRelative.getDegrees();
    double currentRobotFrameDegSigned = mechanismToRobotDeg(currentDegSigned);
    double fieldAngleDegSigned = normalizeToSigned180(lastFieldAngle.getDegrees());
    double robotAngleDegSigned = normalizeToSigned180(getPose.get().getRotation().getDegrees());
    DashboardTelemetry.putNumber(
        "Turret/EncoderRot",
        (turretMotor.getEncoder().getPosition() / Constants.Turret.TURRET_GEAR_RATIO) * 360);
    DashboardTelemetry.putNumber("Turret/CurrentDeg", currentDegSigned);
    DashboardTelemetry.putNumber("Turret/CurrentRobotFrameDeg", currentRobotFrameDegSigned);
    DashboardTelemetry.putNumber("Turret/FieldRelDeg", fieldAngleDegSigned);
    DashboardTelemetry.putNumber("Turret/RobotRelDeg", robotAngleDegSigned);
    DashboardTelemetry.putNumber("Turret/SetpointDeg", setPoint);
    DashboardTelemetry.putNumber("Turret/SetpointCalcDeg", calcSetpointDegSigned);
    DashboardTelemetry.putString(
        "Turret/Target", String.format("(%.2f,%.2f)", lastTarget.getX(), lastTarget.getY()));
  }

  private static double normalizeToSigned180(double degrees) {
    double normalized = ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    if (normalized == -180.0) {
      return 180.0;
    }
    return normalized;
  }

  private static double robotToMechanismDeg(double robotDeg) {
    return normalizeToSigned180(robotDeg + getMechanismZeroOffsetDeg());
  }

  private static double mechanismToRobotDeg(double mechanismDeg) {
    return normalizeToSigned180(mechanismDeg - getMechanismZeroOffsetDeg());
  }

  private static double getMechanismZeroOffsetDeg() {
    return RobotBase.isSimulation() ? 0.0 : Constants.Turret.MECHANISM_ZERO_OFFSET_DEG;
  }

  @Override
  public void simulationPeriodic() {
    // Update simulation physics and visualization
    turretPivot.simIterate();
  }
}
