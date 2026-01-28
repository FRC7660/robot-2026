package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Turret {
  private Turret() {}

  public static final double maxX = Units.inchesToMeters(651.22);

  public static final double blueBumpX = Units.inchesToMeters(182.11);

  public static final double redBumpX = Units.inchesToMeters(maxX - 182.11);

  public static Translation2d getTarget(Translation2d turretPosition, Alliance alliance) {
    // Implementation to get target based on turret position
    return new Translation2d(); // Placeholder return
  }

  public static Rotation2d getFieldRelativeAngle(
      Translation2d turretPosition, Translation2d targetPosition) {
    // Implementation to get current turret position
    return new Rotation2d();
  }

  // public static getRobotRelativeAngle(Rotation2d fieldRelativeAngleRotation2d) {
  //   // Implementation to get robot relative angle
  //   return new Rotation2d();
  // }

}
