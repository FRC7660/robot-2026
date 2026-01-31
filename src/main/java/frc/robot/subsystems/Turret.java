package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Turret {
  private Turret() {}

  // Field dimensions / thresholds (converted from inches to meters)
  public static final double MAX_X = Units.inchesToMeters(651.22);
  public static final double BLUE_THRESHOLD_X = Units.inchesToMeters(182.11);
  public static final double RED_THRESHOLD_X = Units.inchesToMeters(469.11);
  public static final double THRESHOLD_Y = Units.inchesToMeters(158.84);

  // Representative centers for each goal area (meters). These are placeholders
  // suitable for computing angles; replace with precise coordinates if available.
  private static final Translation2d HUB_CENTER =
      new Translation2d((BLUE_THRESHOLD_X + RED_THRESHOLD_X) / 2.0, THRESHOLD_Y);
  private static final Translation2d GOAL_R1_CENTER =
      new Translation2d(MAX_X, Units.inchesToMeters(100.0));
  private static final Translation2d GOAL_R2_CENTER =
      new Translation2d(MAX_X, Units.inchesToMeters(220.0));
  private static final Translation2d GOAL_B1_CENTER =
      new Translation2d(0.0, Units.inchesToMeters(100.0));
  private static final Translation2d GOAL_B2_CENTER =
      new Translation2d(0.0, Units.inchesToMeters(220.0));

  /**
   * Decide which goal area to aim at based purely on the robot position and alliance.
   * Mirrors the pseudo-code provided by the user.
   *
   * @param x robot X position (meters)
   * @param y robot Y position (meters)
   * @param alliance robot alliance (Red or Blue)
   * @return the chosen goal area as a Translation2d (representative center)
   */
  public static Translation2d getTarget(double x, double y, Alliance alliance) {
    if (alliance == Alliance.Red) {
      if (x > RED_THRESHOLD_X) {
        return HUB_CENTER;
      } else {
        if (y < THRESHOLD_Y) {
          return GOAL_R1_CENTER;
        } else {
          return GOAL_R2_CENTER;
        }
      }
    } else { // Blue
      if (x < BLUE_THRESHOLD_X) {
        return HUB_CENTER;
      } else {
        if (y < THRESHOLD_Y) {
          return GOAL_B1_CENTER;
        } else {
          return GOAL_B2_CENTER;
        }
      }
    }
  }

  /** Backwards-compatible overload that accepts a Translation2d robot position. */
  public static Translation2d getTarget(Translation2d turretPosition, Alliance alliance) {
    return getTarget(turretPosition.getX(), turretPosition.getY(), alliance);
  }

  /**
   * Compute the field-relative angle (radians) from turret position to target
   * position.
   */
  public static Rotation2d getFieldRelativeAngle(
      Translation2d turretPosition, Translation2d targetPosition) {
    double dx = targetPosition.getX() - turretPosition.getX();
    double dy = targetPosition.getY() - turretPosition.getY();
    return new Rotation2d(Math.atan2(dy, dx));
  }

  /**
   * Convert a field-relative desired turret angle into a robot-relative turret
   * angle. This returns the angle the turret must take relative to the robot's
   * forward direction. This is a pure computation; it does not command the Neo.
   */
  public static Rotation2d moveTurret(Rotation2d fieldRelativeAngle, Rotation2d robotAngle) {
    double turretRadians = fieldRelativeAngle.getRadians() - robotAngle.getRadians();
    // normalize to [-pi, pi]
    turretRadians = Math.atan2(Math.sin(turretRadians), Math.cos(turretRadians));
    return new Rotation2d(turretRadians);
  }

}
