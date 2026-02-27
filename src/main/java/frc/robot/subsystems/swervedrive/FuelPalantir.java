package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import java.util.Map;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Pure-logic decision layer for the FuelPalantir fuel-collection behavior. */
public class FuelPalantir {

  // ── Constants ─────────────────────────────────────────────────────────────

  static final double FUEL_APPROACH_STOP_AREA = 4.5;
  static final double FUEL_APPROACH_MIN_FORWARD_MPS = 0.10;
  static final double FUEL_APPROACH_MAX_FORWARD_MPS = 0.35;
  static final double FUEL_APPROACH_AREA_P_GAIN = 0.08;
  static final double FUEL_PALANTIR_AUTONOMOUS_TIMEOUT_SEC = 15.0;
  static final double FUEL_NO_TARGET_RELEASE_SEC = 3.0;
  static final double FUEL_CRUISE_FORWARD_MPS = 0.18;
  static final double FUEL_TRACK_YAW_DEADBAND_DEG = 3.0;
  static final double FUEL_TRACK_MAX_ROTATION_RAD_PER_SEC = Math.toRadians(30.0);
  static final double FUEL_TRACK_ROTATION_GAIN = 0.04;
  static final double MAX_FORWARD_STEP_MPS_PER_CYCLE = 0.06;
  static final double MAX_ROTATION_STEP_RAD_PER_SEC_PER_CYCLE = 0.20;

  // ── Records & Enums ───────────────────────────────────────────────────────

  public enum FuelPalantirMode {
    AUTONOMOUS, // 15s timeout, completes on its own
    TELEOP // No timeout (runs until cancelled by button release)
  }

  public record FuelPalantirState(
      Optional<Cameras> lockedCamera,
      Optional<Cameras> preferredCamera,
      double noTargetSinceSec,
      double lastForwardMps,
      double lastRotationRadPerSec,
      boolean hasSeenTarget) {
    public FuelPalantirState(Optional<Cameras> lockedCamera) {
      this(lockedCamera, lockedCamera, Double.NaN, 0.0, 0.0, false);
    }

    public FuelPalantirState(
        Optional<Cameras> lockedCamera,
        double noTargetSinceSec,
        double lastForwardMps,
        double lastRotationRadPerSec) {
      this(
          lockedCamera,
          lockedCamera,
          noTargetSinceSec,
          lastForwardMps,
          lastRotationRadPerSec,
          false);
    }

    public FuelPalantirState(
        Optional<Cameras> lockedCamera,
        double noTargetSinceSec,
        double lastForwardMps,
        double lastRotationRadPerSec,
        boolean hasSeenTarget) {
      this(
          lockedCamera,
          lockedCamera,
          noTargetSinceSec,
          lastForwardMps,
          lastRotationRadPerSec,
          hasSeenTarget);
    }
  }

  public record FuelPalantirStep(
      FuelPalantirState nextState,
      double forwardMps,
      double rotationRadPerSec,
      boolean completed,
      String reason) {}

  // ── Pure static helpers ───────────────────────────────────────────────────

  static Optional<PhotonTrackedTarget> getClosestNonFiducialTarget(Vision.CameraSnapshot snapshot) {
    if (snapshot == null
        || snapshot.latestResult() == null
        || !snapshot.latestResult().hasTargets()) {
      return Optional.empty();
    }
    PhotonTrackedTarget best = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
      if (target.getFiducialId() > 0) {
        continue;
      }
      double absYaw = Math.abs(target.getYaw());
      if (absYaw < minAbsYaw) {
        minAbsYaw = absYaw;
        best = target;
      }
    }
    return Optional.ofNullable(best);
  }

  static Optional<Cameras> chooseLockCamera(
      Map<Cameras, Vision.CameraSnapshot> cameraData, Optional<Cameras> currentlyLocked) {
    if (currentlyLocked.isPresent() && currentlyLocked.get() == Cameras.FRONT_CAMERA) {
      Optional<PhotonTrackedTarget> lockedTarget =
          getClosestNonFiducialTarget(cameraData.get(currentlyLocked.get()));
      if (lockedTarget.isPresent()) {
        return currentlyLocked;
      }
      // Lock released: locked camera no longer sees any fuel
    }
    Cameras[] candidates = {Cameras.FRONT_CAMERA};
    Cameras bestCamera = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (Cameras camera : candidates) {
      Optional<PhotonTrackedTarget> target = getClosestNonFiducialTarget(cameraData.get(camera));
      if (target.isEmpty()) {
        continue;
      }
      double absYaw = Math.abs(target.get().getYaw());
      if (absYaw < minAbsYaw) {
        minAbsYaw = absYaw;
        bestCamera = camera;
      }
    }
    return Optional.ofNullable(bestCamera);
  }

  // ── Core decision function ────────────────────────────────────────────────

  public static FuelPalantirStep fuelPalantir(
      Map<Cameras, Vision.CameraSnapshot> cameraData,
      FuelPalantirState currentState,
      FuelPalantirMode mode,
      double elapsedSec) {

    Optional<Cameras> preferredCamera = currentState.preferredCamera();
    Optional<Cameras> lockedCamera = currentState.lockedCamera();
    double noTargetSinceSec = currentState.noTargetSinceSec();

    if (preferredCamera.isEmpty()) {
      Optional<Cameras> acquired = chooseLockCamera(cameraData, Optional.empty());
      preferredCamera = acquired;
      lockedCamera = acquired;
      if (acquired.isPresent()) {
        noTargetSinceSec = Double.NaN;
      }
    } else {
      // Once we choose a side, stay with that side until it has no balls for 3s.
      lockedCamera = preferredCamera;
    }

    Optional<PhotonTrackedTarget> target =
        lockedCamera.flatMap(camera -> getClosestNonFiducialTarget(cameraData.get(camera)));
    boolean hasSeenTarget = currentState.hasSeenTarget() || target.isPresent();
    if (preferredCamera.isPresent()) {
      if (target.isPresent()) {
        noTargetSinceSec = Double.NaN;
      } else if (Double.isNaN(noTargetSinceSec)) {
        noTargetSinceSec = elapsedSec;
      } else if ((elapsedSec - noTargetSinceSec) >= FUEL_NO_TARGET_RELEASE_SEC) {
        preferredCamera = Optional.empty();
        lockedCamera = Optional.empty();
        noTargetSinceSec = Double.NaN;
      }
    }

    // Search spin is only used before first acquisition. After first acquisition, keep heading
    // straight and keep driving on the chosen side until no balls are seen for 3 seconds.
    double desiredRotation = hasSeenTarget ? 0.0 : SwerveSubsystem.SEARCH_ROTATION_RAD_PER_SEC;
    double desiredForward = 0.0;

    if (target.isPresent()) {
      double yawDeg = target.get().getYaw();
      if (Math.abs(yawDeg) <= FUEL_TRACK_YAW_DEADBAND_DEG) {
        desiredRotation = 0.0;
      } else {
        desiredRotation =
            MathUtil.clamp(
                -Math.toRadians(yawDeg) * FUEL_TRACK_ROTATION_GAIN,
                -FUEL_TRACK_MAX_ROTATION_RAD_PER_SEC,
                FUEL_TRACK_MAX_ROTATION_RAD_PER_SEC);
      }
      double area = target.get().getArea();
      if (area >= FUEL_APPROACH_STOP_AREA) {
        desiredForward = 0.0;
      } else {
        double areaError = FUEL_APPROACH_STOP_AREA - area;
        desiredForward =
            MathUtil.clamp(
                areaError * FUEL_APPROACH_AREA_P_GAIN,
                FUEL_APPROACH_MIN_FORWARD_MPS,
                FUEL_APPROACH_MAX_FORWARD_MPS);
      }
    } else if (preferredCamera.isPresent()) {
      desiredForward = FUEL_CRUISE_FORWARD_MPS;
    }
    double forward =
        currentState.lastForwardMps()
            + MathUtil.clamp(
                desiredForward - currentState.lastForwardMps(),
                -MAX_FORWARD_STEP_MPS_PER_CYCLE,
                MAX_FORWARD_STEP_MPS_PER_CYCLE);
    double rotation =
        currentState.lastRotationRadPerSec()
            + MathUtil.clamp(
                desiredRotation - currentState.lastRotationRadPerSec(),
                -MAX_ROTATION_STEP_RAD_PER_SEC_PER_CYCLE,
                MAX_ROTATION_STEP_RAD_PER_SEC_PER_CYCLE);

    boolean timedOut =
        mode == FuelPalantirMode.AUTONOMOUS && elapsedSec >= FUEL_PALANTIR_AUTONOMOUS_TIMEOUT_SEC;
    boolean completed = timedOut;
    String reason = timedOut ? "timeout" : "searching";

    return new FuelPalantirStep(
        new FuelPalantirState(
            lockedCamera, preferredCamera, noTargetSinceSec, forward, rotation, hasSeenTarget),
        forward,
        rotation,
        completed,
        reason);
  }

  private FuelPalantir() {}
}
