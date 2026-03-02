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
            TurretHelpers.GOAL_X_RED,
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
            TurretHelpers.GOAL_X_RED,
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
            TurretHelpers.GOAL_X_RED,
            TurretHelpers.GOAL_Y_LOW),
        // D: (325.61, 79.423) Blue -> GOAL_B1
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(79.423),
            Alliance.Blue,
            TurretHelpers.GOAL_X_BLUE,
            TurretHelpers.GOAL_Y_LOW),

        // E: (325.61, 238.263)
        // Red -> GOAL_R2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Red,
            TurretHelpers.GOAL_X_RED,
            TurretHelpers.GOAL_Y_HIGH),
        // E: (325.61, 238.263) Blue -> GOAL_B2
        Arguments.of(
            Units.inchesToMeters(325.61),
            Units.inchesToMeters(238.263),
            Alliance.Blue,
            TurretHelpers.GOAL_X_BLUE,
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
            TurretHelpers.GOAL_X_BLUE,
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
            TurretHelpers.GOAL_X_BLUE,
            TurretHelpers.GOAL_Y_HIGH));
  }

  @ParameterizedTest
  @MethodSource("zeroPointProvider")
  void turretZeroPoint_tableDriven(double currentDeg, double idealDeg, double expectedFinalDeg) {
    TurretZeroPoint z = new TurretZeroPoint(); // not latched initially per test spec
    double delta = z.updateAndCompute(idealDeg, currentDeg);

    // compute final absolute angle and normalize to [0,360)
    double finalDeg = (currentDeg + delta) % 360.0;
    if (finalDeg < 0) {
      finalDeg += 360.0;
    }

    assertEquals(expectedFinalDeg, finalDeg, 1e-3);
  }

  static Stream<Arguments> zeroPointProvider() {
    return Stream.of(
        // Target within sticky band AND zero lies on shortest path -> command to zero.
        Arguments.of(350.0, 15.0, 0.0),
        Arguments.of(340.0, 10.0, 0.0),
        Arguments.of(355.0, 20.0, 0.0),

        // Target within sticky band BUT zero not on shortest path -> command toward target.
        Arguments.of(30.0, 20.0, 20.0),
        Arguments.of(59.0, 12.0, 12.0),
        Arguments.of(88.0, 19.0, 19.0),

        // Long Route Test Cases
        Arguments.of(340.0, 30.0, 30.0),
        Arguments.of(340.0, 21.0, 21.0),
        Arguments.of(350.0, 40.0, 40.0),

        // Short Route Test Cases
        Arguments.of(230.0, 200.0, 200.0),
        Arguments.of(214.0, 235.0, 235.0),
        Arguments.of(318.0, 355.0, 355.0));
  }
}
