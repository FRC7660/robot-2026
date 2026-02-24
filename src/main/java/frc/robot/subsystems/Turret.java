package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers;
import frc.robot.lib.TurretZeroPoint;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/** Simple turret subsystem that holds a NEO Vortex and a supplier for the robot pose. */
public class Turret extends SubsystemBase {
  private final SparkFlex turretMotor =
      new SparkFlex(Constants.Turret.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final Supplier<Pose2d> getPose;
  // YAMS controller + mechanism
  private final SmartMotorController turretSmartMotorController;
  private final Pivot turretPivot;
  // Most recent setpoint (robot-relative radians). Useful for debugging and tests.
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
            // Feedback Constants (PID Constants)
            .withClosedLoopController(
                Constants.Turret.TURRET_P,
                Constants.Turret.TURRET_I,
                Constants.Turret.TURRET_D,
                DegreesPerSecond.of(180),
                DegreesPerSecondPerSecond.of(90))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimClosedLoopController(
                .15, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("TurretMotorConfig", TelemetryVerbosity.HIGH)
            // Gearing from motor rotor to turret.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    this.turretSmartMotorController = new SparkWrapper(turretMotor, DCMotor.getNEO(1), smcConfig);

    PivotConfig pivotConfig =
        new PivotConfig(turretSmartMotorController)
            .withHardLimit(Degrees.of(0), Degrees.of(270))
            .withStartingPosition(Degrees.of(135))
            .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
            .withMOI(Meters.of(0.254), Pounds.of(2));

    this.turretPivot = new Pivot(pivotConfig);
    this.zeroPoint = new TurretZeroPoint();
  }

  /**
   * Set the turret desired angle relative to the robot forward direction. This is a stub that
   * records the setpoint; convert to motor commands in a follow-up change.
   *
   * @param robotRelativeAngle turret angle relative to robot forward
   */
  public void setTurretSetpoint(Rotation2d robotRelativeAngle) {
    this.lastSetpoint = robotRelativeAngle;

    // Compute desired absolute turret angle (field-agnostic in robot frame)
    Pose2d pose = getPose.get();
    double desiredAbsRad = pose.getRotation().getRadians() + robotRelativeAngle.getRadians();

    // Convert desired absolute angle (radians) to degrees [0,360)
    double desiredAbsDeg = Math.toDegrees(desiredAbsRad) % 360.0;
    if (desiredAbsDeg < 0) {
      desiredAbsDeg += 360.0;
    }
    // Enforce turret hard range of motion: [0, 270] degrees absolute
    desiredAbsDeg = Math.max(0.0, Math.min(270.0, desiredAbsDeg));
    this.lastDesiredAbsDeg = desiredAbsDeg;

    // Read current motor rotations from encoder (REV API: getEncoder().getPosition())
    double currentMotorRotations = 0.0;
    try {
      currentMotorRotations = turretMotor.getEncoder().getPosition();
    } catch (Throwable t) {
      currentMotorRotations = 0.0;
    }

    // Convert current motor rotations to turret absolute angle degrees
    double currentTurretRotations = currentMotorRotations / Constants.Turret.TURRET_GEAR_RATIO;
    double currentAngleDeg = (currentTurretRotations * 360.0) % 360.0;
    if (currentAngleDeg < 0) {
      currentAngleDeg += 360.0;
    }

    // Sticky-zero behavior with zero-point regulator
    double rotationCommandDeg = zeroPoint.updateAndCompute(desiredAbsDeg, currentAngleDeg);
    this.lastRotationCommandDeg = rotationCommandDeg;

    // Compute practical absolute setpoint based on current angle + commanded delta
    double practicalAbsDeg = currentAngleDeg + rotationCommandDeg;
    if (practicalAbsDeg < 0) {
      practicalAbsDeg += 360.0;
    }
    practicalAbsDeg = Math.max(0.0, Math.min(270.0, practicalAbsDeg));

    turretPivot.setMechanismPositionSetpoint(Degrees.of(practicalAbsDeg));
  }

  /** Get the most recent turret setpoint (robot-relative angle). Primarily useful for testing. */
  public Rotation2d getLastSetpoint() {
    return lastSetpoint;
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
    SmartDashboard.putNumber(
        "Turret/Position",
        (turretMotor.getEncoder().getPosition() / Constants.Turret.TURRET_GEAR_RATIO) * 360);
    SmartDashboard.putNumber(
        "Turret/Setpoint (Robot Relative)", getRobotRelativeAngle().getDegrees());
        turretPivot.updateTelemetry();
  }
  
  @Override
  public void simulationPeriodic() {
    // Update simulation physics and visualization
    turretPivot.simIterate();
  }
}

