package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class TurretTest {

  @ParameterizedTest
  @MethodSource("targetProvider")
  void getTarget_tableDriven(
      double turretX, double turretY, Alliance alliance, double expX, double expY) {
    Translation2d turretPos = new Translation2d(turretX, turretY);
    Translation2d target = Turret.getTarget(turretPos, alliance);
    assertNotNull(target);
    assertEquals(expX, target.getX(), 1e-9);
    assertEquals(expY, target.getY(), 1e-9);
  }

  static Stream<Arguments> targetProvider() {
    return Stream.of(
        // A: (78.305, 79.423)
        // Red -> GOAL_R1
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(79.423),
            Alliance.Red,
            Turret.MAX_X,
            Units.inchesToMeters(100.0)),
        // A: (78.305, 79.423) Blue -> HUB
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            (Turret.BLUE_THRESHOLD_X + Turret.RED_THRESHOLD_X) / 2.0,
            Turret.THRESHOLD_Y),

        // C: (78.305, 238.263)
        // Red -> GOAL_R2
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            Turret.MAX_X,
            Units.inchesToMeters(220.0)),
        // C: (78.305, 238.263) Blue -> HUB
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            (Turret.BLUE_THRESHOLD_X + Turret.RED_THRESHOLD_X) / 2.0,
            Turret.THRESHOLD_Y),

        // D: (325.61, 79.423)
        // Red -> GOAL_R1
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(79.423),
            Alliance.Red,
            Turret.MAX_X,
            Units.inchesToMeters(100.0)),
        // D: (325.61, 79.423) Blue -> GOAL_B1
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            0.0,
            Units.inchesToMeters(100.0)),

        // E: (325.61, 238.263)
        // Red -> GOAL_R2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            Turret.MAX_X,
            Units.inchesToMeters(220.0)),
        // E: (325.61, 238.263) Blue -> GOAL_B2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            0.0,
            Units.inchesToMeters(220.0)),

        // F: (560.165, 79.423)
        // Red -> HUB
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(79.423),
            Alliance.Red,
            (Turret.BLUE_THRESHOLD_X + Turret.RED_THRESHOLD_X) / 2.0,
            Turret.THRESHOLD_Y),
        // F: (560.165, 79.423) Blue -> GOAL_B1
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            0.0,
            Units.inchesToMeters(100.0)),

        // H: (560.165, 238.263)
        // Red -> HUB
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            (Turret.BLUE_THRESHOLD_X + Turret.RED_THRESHOLD_X) / 2.0,
            Turret.THRESHOLD_Y),
        // H: (560.165, 238.263) Blue -> GOAL_B2
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            0.0,
            Units.inchesToMeters(220.0)));
  }
}
