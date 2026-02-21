package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TurretHelpers;
import java.util.function.Supplier;

/** Simple turret subsystem that holds a Neo and a supplier for the robot pose. */
public class Turret extends SubsystemBase {
  private final SparkMax turretMotor =
      new SparkMax(Constants.Turret.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final Supplier<Pose2d> getPose;

  // Most recent setpoint (robot-relative radians). Useful for debugging and tests.
  private Rotation2d lastSetpoint = new Rotation2d();
  // Most recently chosen target (field coordinates, meters)
  private Translation2d lastTarget = new Translation2d();

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
    System.out.println("[BootTrace] Turret ctor start");
    this.getPose = getPose;
    System.out.println("[BootTrace] Turret configureTurretMotor start");
    SparkMaxConfig turretConfig = configureTurretMotor();
    System.out.println("[BootTrace] Turret motor configure start");
    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("[BootTrace] Turret ctor complete");
  }

  // Turret Motor Config
  public SparkMaxConfig configureTurretMotor() {
    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig.idleMode(IdleMode.kBrake).inverted(false);
    return turretConfig;
  }

  /**
   * Set the turret desired angle relative to the robot forward direction. This is a stub that
   * records the setpoint; convert to motor commands in a follow-up change.
   *
   * @param robotRelativeAngle turret angle relative to robot forward
   */
  public void setTurretSetpoint(Rotation2d robotRelativeAngle) {
    this.lastSetpoint = robotRelativeAngle;
    // TODO: convert Rotation2d to motor position/velocity and command the Neo here.
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
    Translation2d robotPos = pose.getTranslation();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Translation2d target = TurretHelpers.getTarget(robotPos, alliance);
    // record the chosen target for external inspection / dashboard
    this.lastTarget = target;
    Rotation2d fieldAngle = TurretHelpers.getFieldRelativeAngle(robotPos, target);
    Rotation2d robotRelative = TurretHelpers.getRobotRelative(fieldAngle, pose.getRotation());
    return robotRelative;
  }

  /**
   * Returns the last computed target translation (field coordinates).
   *
   * @return last chosen {@link Translation2d} target
   */
  public Translation2d getLastTarget() {
    return lastTarget;
  }
}
