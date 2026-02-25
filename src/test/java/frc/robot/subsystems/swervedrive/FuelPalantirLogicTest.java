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

  @Test
  void fuelPalantir_keepsLockDuringBriefDetectionDropout() {
    Map<Cameras, Vision.CameraSnapshot> noTargets = baseCameraData();
    FuelPalantir.FuelPalantirState state =
        new FuelPalantir.FuelPalantirState(Optional.of(Cameras.BACK_CAMERA));

    FuelPalantir.FuelPalantirStep step1 =
        FuelPalantir.fuelPalantir(
            noTargets, state, FuelPalantir.FuelPalantirMode.TELEOP, 0.1);
    FuelPalantir.FuelPalantirStep step2 =
        FuelPalantir.fuelPalantir(
            noTargets, step1.nextState(), FuelPalantir.FuelPalantirMode.TELEOP, 0.2);

    assertEquals(Optional.of(Cameras.BACK_CAMERA), step1.nextState().lockedCamera());
    assertEquals(Optional.of(Cameras.BACK_CAMERA), step2.nextState().lockedCamera());
  }

  @Test
  void fuelPalantir_releasesLockAfterThreeSecondsWithoutTargets() {
    Map<Cameras, Vision.CameraSnapshot> noTargets = baseCameraData();
    FuelPalantir.FuelPalantirState state =
        new FuelPalantir.FuelPalantirState(Optional.of(Cameras.BACK_CAMERA));

    FuelPalantir.FuelPalantirStep step1 =
        FuelPalantir.fuelPalantir(
            noTargets, state, FuelPalantir.FuelPalantirMode.TELEOP, 0.1);
    FuelPalantir.FuelPalantirStep step2 =
        FuelPalantir.fuelPalantir(
            noTargets, step1.nextState(), FuelPalantir.FuelPalantirMode.TELEOP, 2.9);
    FuelPalantir.FuelPalantirStep step3 =
        FuelPalantir.fuelPalantir(
            noTargets, step2.nextState(), FuelPalantir.FuelPalantirMode.TELEOP, 3.2);

    assertEquals(Optional.of(Cameras.BACK_CAMERA), step1.nextState().lockedCamera());
    assertEquals(Optional.of(Cameras.BACK_CAMERA), step2.nextState().lockedCamera());
    assertTrue(step3.nextState().lockedCamera().isEmpty());
  }

  @Test
  void fuelPalantir_slewLimitsForwardCommand() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 0.0, 0.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(Optional.empty()),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.1);

    assertEquals(FuelPalantir.MAX_FORWARD_STEP_MPS_PER_CYCLE, step.forwardMps(), 1e-9);
  }

  @Test
  void fuelPalantir_ballSeenNearCenter_stopsRotationSearch() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 1.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(Optional.empty(), 0, 0.0, 0.0),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.1);

    assertEquals(0.0, step.rotationRadPerSec(), 1e-9);
  }

  @Test
  void fuelPalantir_ballSeenOffCenter_usesTrackingNotSearchRotation() {
    Map<Cameras, Vision.CameraSnapshot> data = baseCameraData();
    data.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 25.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep step =
        FuelPalantir.fuelPalantir(
            data,
            new FuelPalantir.FuelPalantirState(Optional.empty(), 0, 0.0, 0.0),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.1);

    assertTrue(step.rotationRadPerSec() < 0.0);
    assertTrue(
        Math.abs(step.rotationRadPerSec()) < SwerveSubsystem.SEARCH_ROTATION_RAD_PER_SEC,
        "Tracking rotation should be below search spin rate");
  }

  @Test
  void fuelPalantir_afterSeeingBall_doesNotReturnToSearchSpinWhenLost() {
    Map<Cameras, Vision.CameraSnapshot> seenData = baseCameraData();
    seenData.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 12.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep seenStep =
        FuelPalantir.fuelPalantir(
            seenData,
            new FuelPalantir.FuelPalantirState(Optional.empty(), 0, 0.0, 0.0, false),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.1);

    FuelPalantir.FuelPalantirStep lostStep =
        FuelPalantir.fuelPalantir(
            baseCameraData(),
            seenStep.nextState(),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.2);

    assertTrue(seenStep.nextState().hasSeenTarget());
    assertEquals(0.0, lostStep.rotationRadPerSec(), 1e-9);
  }

  @Test
  void fuelPalantir_afterLockingBackCamera_doesNotSwitchToFrontCamera() {
    Map<Cameras, Vision.CameraSnapshot> backOnly = baseCameraData();
    backOnly.put(Cameras.BACK_CAMERA, snapshotWithTarget(-1, 10.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep backStep =
        FuelPalantir.fuelPalantir(
            backOnly,
            new FuelPalantir.FuelPalantirState(Optional.empty(), 0, 0.0, 0.0, false),
            FuelPalantir.FuelPalantirMode.TELEOP,
            0.1);
    assertEquals(Optional.of(Cameras.BACK_CAMERA), backStep.nextState().lockedCamera());

    Map<Cameras, Vision.CameraSnapshot> frontOnly = baseCameraData();
    frontOnly.put(Cameras.FRONT_CAMERA, snapshotWithTarget(Cameras.FRONT_CAMERA, -1, 5.0, 1.0, 1.0));

    FuelPalantir.FuelPalantirStep switchedStep =
        FuelPalantir.fuelPalantir(
            frontOnly, backStep.nextState(), FuelPalantir.FuelPalantirMode.TELEOP, 0.2);

    assertNotEquals(Optional.of(Cameras.FRONT_CAMERA), switchedStep.nextState().lockedCamera());
  }
}
