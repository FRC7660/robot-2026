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
      Cameras camera, int fiducialId, double yawDeg, double area, double timestampSec) {
    PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
    when(target.getFiducialId()).thenReturn(fiducialId);
    when(target.getYaw()).thenReturn(yawDeg);
    when(target.getArea()).thenReturn(area);

    PhotonPipelineResult latest = mock(PhotonPipelineResult.class);
    when(latest.hasTargets()).thenReturn(true);
    when(latest.getTargets()).thenReturn(List.of(target));
    when(latest.getTimestampSeconds()).thenReturn(timestampSec);

    return new Vision.CameraSnapshot(camera, List.of(), latest);
  }

  private static Vision.CameraSnapshot snapshotWithTarget(
      int fiducialId, double yawDeg, double area, double timestampSec) {
    return snapshotWithTarget(Cameras.BACK_CAMERA, fiducialId, yawDeg, area, timestampSec);
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
    map.put(Cameras.BACK_CAMERA, emptySnapshot(Cameras.BACK_CAMERA));
    map.put(Cameras.FRONT_CAMERA, emptySnapshot(Cameras.FRONT_CAMERA));
    return map;
  }

  @Test
  void fuelPalantir_locksCameraAndCommandsMotionWhenFuelSeen() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 10.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            1.0);

    assertTrue(step.nextState().lockedCamera().isPresent());
    assertEquals(Cameras.BACK_CAMERA, step.nextState().lockedCamera().get());
    assertTrue(step.forwardMps() > 0.0);
    assertFalse(step.fuelCollectedThisCycle());
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_incrementsProxyCountWhenTargetAreaReached() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 1.0, 5.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(2, Optional.of(Cameras.BACK_CAMERA), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            2.0);

    assertTrue(step.fuelCollectedThisCycle());
    assertEquals(3, step.nextState().proxyCollectedFuelCount());
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_continueModeTimeoutCompletes() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            30.0);

    assertTrue(step.completed());
    assertEquals("timeout", step.reason());
  }

  @Test
  void fuelPalantir_stopModeTimeoutCompletes() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(0, Optional.empty(), false),
            FuelPalantir.FuelPalantirMode.STOP_AFTER_20S,
            20.0);

    assertTrue(step.completed());
    assertEquals("timeout", step.reason());
  }

  @Test
  void fuelPalantir_completesWhenProxyFuelTargetReached() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 0.0, 6.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(7, Optional.of(Cameras.BACK_CAMERA), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            5.0);

    assertTrue(step.completed());
    assertEquals("target_fuel_count_reached", step.reason());
    assertEquals(8, step.nextState().proxyCollectedFuelCount());
  }

  @Test
  void fuelPalantir_doesNotDoubleCountWhenAreaStaysAboveThreshold() {
    // First cycle: was below → now above → counts as 1 collection
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 0.0, 5.0, 1.0));

    FuelPalantir.FuelPalantirStep step1 =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(0, Optional.of(Cameras.BACK_CAMERA), false),
            FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S,
            1.0);

    assertTrue(step1.fuelCollectedThisCycle());
    assertEquals(1, step1.nextState().proxyCollectedFuelCount());
    assertTrue(step1.nextState().wasAboveAreaThreshold());

    // Second cycle: still above threshold with same piece — must NOT count again
    FuelPalantir.FuelPalantirStep step2 =
        FuelPalantir.fuelPalantir(
            data, step1.nextState(), FuelPalantir.FuelPalantirMode.CONTINUE_AFTER_30S, 1.02);

    assertFalse(step2.fuelCollectedThisCycle());
    assertEquals(1, step2.nextState().proxyCollectedFuelCount());
  }

  @Test
  void chooseLockCamera_releasesLockWhenLockedCameraLosesTarget() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    // BACK_CAMERA is locked but has no targets; FRONT_CAMERA has a fuel target
    data.put(Cameras.FRONT_CAMERA, snapshotWithTarget(Cameras.FRONT_CAMERA, -1, 5.0, 1.0, 1.0));

    Optional<Cameras> result =
        FuelPalantir.chooseLockCamera(data, Optional.of(Cameras.BACK_CAMERA));

    assertTrue(result.isPresent());
    assertEquals(Cameras.FRONT_CAMERA, result.get());
  }

  @Test
  void chooseLockCamera_keepsLockWhenLockedCameraStillSeesFuel() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(Cameras.BACK_CAMERA, -1, 3.0, 2.0, 1.0));
    data.put(Cameras.FRONT_CAMERA, snapshotWithTarget(Cameras.FRONT_CAMERA, -1, 1.0, 3.0, 1.0));

    Optional<Cameras> result =
        FuelPalantir.chooseLockCamera(data, Optional.of(Cameras.BACK_CAMERA));

    assertTrue(result.isPresent());
    assertEquals(Cameras.BACK_CAMERA, result.get());
  }
}
