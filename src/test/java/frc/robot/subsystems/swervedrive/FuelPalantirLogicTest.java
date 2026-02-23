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
            new FuelPalantir.FuelPalantirState(Optional.empty()),
            FuelPalantir.FuelPalantirMode.AUTONOMOUS,
            1.0);

    assertTrue(step.nextState().lockedCamera().isPresent());
    assertEquals(Cameras.BACK_CAMERA, step.nextState().lockedCamera().get());
    assertTrue(step.forwardMps() > 0.0);
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_stopsForwardWhenAreaAboveThreshold() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 1.0, 5.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(Optional.of(Cameras.BACK_CAMERA)),
            FuelPalantir.FuelPalantirMode.AUTONOMOUS,
            2.0);

    assertEquals(0.0, step.forwardMps());
    assertFalse(step.completed());
  }

  @Test
  void fuelPalantir_autonomousTimeoutCompletes() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(Optional.empty()),
            FuelPalantir.FuelPalantirMode.AUTONOMOUS,
            15.0);

    assertTrue(step.completed());
    assertEquals("timeout", step.reason());
  }

  @Test
  void fuelPalantir_teleopNeverCompletesOnItsOwn() {
    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            new FuelPalantir.FuelPalantirState(Optional.empty()),
            FuelPalantir.FuelPalantirMode.TELEOP,
            600.0);

    assertFalse(step.completed());
    assertEquals("searching", step.reason());
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
