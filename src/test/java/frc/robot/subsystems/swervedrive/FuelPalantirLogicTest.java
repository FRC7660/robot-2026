package frc.robot.subsystems.swervedrive;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class FuelPalantirLogicTest {

  private static Vision.CameraSnapshot snapshotWithTarget(
      int fiducialId, double yawDeg, double area, double timestampSec) {
    PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
    when(target.getFiducialId()).thenReturn(fiducialId);
    when(target.getYaw()).thenReturn(yawDeg);
    when(target.getArea()).thenReturn(area);

    PhotonPipelineResult latest = mock(PhotonPipelineResult.class);
    when(latest.hasTargets()).thenReturn(true);
    when(latest.getTargets()).thenReturn(List.of(target));
    when(latest.getTimestampSeconds()).thenReturn(timestampSec);

    return new Vision.CameraSnapshot(Cameras.CAMERA0, List.of(), latest);
  }

  private static Vision.CameraSnapshot emptySnapshot(Cameras camera) {
    PhotonPipelineResult latest = mock(PhotonPipelineResult.class);
    when(latest.hasTargets()).thenReturn(false);
    when(latest.getTargets()).thenReturn(List.of());
    when(latest.getTimestampSeconds()).thenReturn(0.0);
    return new Vision.CameraSnapshot(camera, List.of(), latest);
  }

  private static Map<Cameras, Vision.CameraSnapshot> baseCameraData() {
    Map<Cameras, Vision.CameraSnapshot> map = new EnumMap<>(Cameras.class);
    map.put(Cameras.CAMERA0, emptySnapshot(Cameras.CAMERA0));
    map.put(Cameras.CAMERA1, emptySnapshot(Cameras.CAMERA1));
    return map;
  }

  @Test
  void fuelPalantir_locksCameraAndCommandsMotionWhenFuelSeen() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.CAMERA0, snapshotWithTarget(-1, 10.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            1.0);

    assertTrue(step.nextState().lockedCamera().isPresent());
    assertEquals(Cameras.CAMERA0, step.nextState().lockedCamera().get());
    assertTrue(step.forwardMps() > 0.0);
    assertFalse(step.fuelCollectedThisCycle());
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_incrementsProxyCountWhenTargetAreaReached() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.CAMERA0, snapshotWithTarget(-1, 1.0, 5.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(2, Optional.of(Cameras.CAMERA0), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            2.0);

    assertTrue(step.fuelCollectedThisCycle());
    assertEquals(3, step.nextState().proxyCollectedFuelCount());
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_continueModeTimeoutCompletesWithoutHold() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            30.0);

    assertTrue(step.completed());
    assertFalse(step.nextState().holdAfterCompletion());
    assertEquals("timeout", step.reason());
  }

  @Test
  void fuelPalantir_stopModeTimeoutCompletesWithHold() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.STOP_AFTER_20S,
            20.0);

    assertTrue(step.completed());
    assertTrue(step.nextState().holdAfterCompletion());
    assertEquals("timeout", step.reason());
  }

  @Test
  void fuelPalantir_completesWhenProxyFuelTargetReached() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.CAMERA0, snapshotWithTarget(-1, 0.0, 6.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(7, Optional.of(Cameras.CAMERA0), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            5.0);

    assertTrue(step.completed());
    assertEquals("target_fuel_count_reached", step.reason());
    assertEquals(8, step.nextState().proxyCollectedFuelCount());
  }
}
