package frc.robot.lib;

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
    Translation2d target = TurretHelpers.getTarget(turretPos, alliance);
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
            TurretHelpers.MAX_X,
            TurretHelpers.GOAL_Y_LOW),
        // A: (78.305, 79.423) Blue -> HUB
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            TurretHelpers.HUB_CENTER_BLUE.getX(),
            TurretHelpers.HUB_CENTER_BLUE.getY()),

        // C: (78.305, 238.263)
        // Red -> GOAL_R2
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            TurretHelpers.MAX_X,
            TurretHelpers.GOAL_Y_HIGH),
        // C: (78.305, 238.263) Blue -> HUB
        Arguments.of(
            Units.inchesToMeters(78.305),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            TurretHelpers.HUB_CENTER_BLUE.getX(),
            TurretHelpers.HUB_CENTER_BLUE.getY()),

        // D: (325.61, 79.423)
        // Red -> GOAL_R1
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(79.423),
            Alliance.Red,
            TurretHelpers.MAX_X,
            TurretHelpers.GOAL_Y_LOW),
        // D: (325.61, 79.423) Blue -> GOAL_B1
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            0.0,
            TurretHelpers.GOAL_Y_LOW),

        // E: (325.61, 238.263)
        // Red -> GOAL_R2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            TurretHelpers.MAX_X,
            TurretHelpers.GOAL_Y_HIGH),
        // E: (325.61, 238.263) Blue -> GOAL_B2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            0.0,
            TurretHelpers.GOAL_Y_HIGH),

        // F: (560.165, 79.423)
        // Red -> HUB
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(79.423),
            Alliance.Red,
            TurretHelpers.HUB_CENTER_RED.getX(),
            TurretHelpers.HUB_CENTER_RED.getY()),
        // F: (560.165, 79.423) Blue -> GOAL_B1
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            0.0,
            TurretHelpers.GOAL_Y_LOW),

        // H: (560.165, 238.263)
        // Red -> HUB
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            TurretHelpers.HUB_CENTER_RED.getX(),
            TurretHelpers.HUB_CENTER_RED.getY()),
        // H: (560.165, 238.263) Blue -> GOAL_B2
        Arguments.of(
            Units.inchesToMeters(560.165),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            0.0,
            TurretHelpers.GOAL_Y_HIGH));
  }
}
