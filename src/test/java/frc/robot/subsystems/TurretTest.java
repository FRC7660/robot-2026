package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.Arguments;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class TurretTest {

  @Test
  void getTarget_returnsZeroTranslation_whenPlaceholder() {
    Translation2d turretPos = new Translation2d(1.0, 2.0);
    Translation2d target = Turret.getTarget(turretPos, Alliance.Blue);
    assertNotNull(target);
    assertEquals(0.0, target.getX(), 1e-9);
    assertEquals(0.0, target.getY(), 1e-9);
  }

  @Test
  void getFieldRelativeAngle_returnsZeroRotation_whenPlaceholder() {
    Translation2d turretPos = new Translation2d(1.0, 2.0);
    Translation2d targetPos = new Translation2d(3.0, 4.0);
    Rotation2d angle = Turret.getFieldRelativeAngle(turretPos, targetPos);
    assertNotNull(angle);
    assertEquals(0.0, angle.getRadians(), 1e-9);
  }

  /**
   * Table-driven test for Turret.getTarget using a MethodSource. Currently the
   * implementation returns a zero Translation2d, so all expected values are (0,0).
   * When the method is implemented, update the expected values in the provider.
   */
  @ParameterizedTest
  @MethodSource("targetProvider")
  void getTarget_tableDriven(double turretX, double turretY, Alliance alliance, double expX, double expY) {
    Translation2d turretPos = new Translation2d(turretX, turretY);
    Translation2d target = Turret.getTarget(turretPos, alliance);
    assertNotNull(target);
    assertEquals(expX, target.getX(), 1e-9);
    assertEquals(expY, target.getY(), 1e-9);
  }

  static Stream<Arguments> targetProvider() {
    return Stream.of(
        // turretX, turretY, alliance, expectedX, expectedY
        Arguments.of(0.0, 0.0, Alliance.Blue, 0.0, 0.0),
        Arguments.of(1.0, 2.0, Alliance.Blue, 0.0, 0.0),
        Arguments.of(-3.5, 4.2, Alliance.Red, 0.0, 0.0),
        Arguments.of(10.0, -8.0, Alliance.Red, 0.0, 0.0)
    );
  }
}
