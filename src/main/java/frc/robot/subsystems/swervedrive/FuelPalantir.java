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

  // ── Records & Enums ───────────────────────────────────────────────────────

  public enum FuelPalantirMode {
    AUTONOMOUS, // 15s timeout, completes on its own
    TELEOP // No timeout (runs until cancelled by button release)
  }

  public record FuelPalantirState(Optional<Cameras> lockedCamera) {}

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
    if (currentlyLocked.isPresent()) {
      Optional<PhotonTrackedTarget> lockedTarget =
          getClosestNonFiducialTarget(cameraData.get(currentlyLocked.get()));
      if (lockedTarget.isPresent()) {
        return currentlyLocked;
      }
      // Lock released: locked camera no longer sees any fuel
    }
    Cameras[] candidates = {Cameras.BACK_CAMERA, Cameras.FRONT_CAMERA};
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

    Optional<Cameras> lockedCamera = chooseLockCamera(cameraData, currentState.lockedCamera());
    Optional<PhotonTrackedTarget> target =
        lockedCamera.flatMap(camera -> getClosestNonFiducialTarget(cameraData.get(camera)));

    double rotation = SwerveSubsystem.SEARCH_ROTATION_RAD_PER_SEC;
    double forward = 0.0;

    if (target.isPresent()) {
      double yawDeg = target.get().getYaw();
      rotation = SwerveSubsystem.calculateRotationFromYawDeg(yawDeg);
      double area = target.get().getArea();
      if (area >= FUEL_APPROACH_STOP_AREA) {
        forward = 0.0;
      } else {
        double areaError = FUEL_APPROACH_STOP_AREA - area;
        forward =
            MathUtil.clamp(
                areaError * FUEL_APPROACH_AREA_P_GAIN,
                FUEL_APPROACH_MIN_FORWARD_MPS,
                FUEL_APPROACH_MAX_FORWARD_MPS);
      }
    }

    boolean timedOut =
        mode == FuelPalantirMode.AUTONOMOUS && elapsedSec >= FUEL_PALANTIR_AUTONOMOUS_TIMEOUT_SEC;
    boolean completed = timedOut;
    String reason = timedOut ? "timeout" : "searching";

    return new FuelPalantirStep(
        new FuelPalantirState(lockedCamera), forward, rotation, completed, reason);
  }

  private FuelPalantir() {}
}
