package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import java.util.Map;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Pure-logic decision layer for group-based fuel collection behavior. */
public class FuelPalantirGroup {

  static final int GROUP_ENTER_TARGET_COUNT = 6; // enter when >5
  static final int GROUP_EXIT_TARGET_COUNT = 4; // exit when <=4
  static final double GROUP_NO_TARGET_RELEASE_SEC = 3.0;
  static final double GROUP_AUTONOMOUS_TIMEOUT_SEC = 15.0;
  static final double GROUP_CRUISE_FORWARD_MPS = 0.22;
  static final double GROUP_APPROACH_STOP_AREA = 4.5;
  static final double GROUP_APPROACH_MIN_FORWARD_MPS = 0.10;
  static final double GROUP_APPROACH_MAX_FORWARD_MPS = 0.35;
  static final double GROUP_APPROACH_AREA_P_GAIN = 0.08;
  static final double GROUP_TRACK_YAW_DEADBAND_DEG = 3.0;
  static final double GROUP_TRACK_MAX_ROTATION_RAD_PER_SEC = Math.toRadians(30.0);
  static final double GROUP_TRACK_ROTATION_GAIN = 0.04;
  static final double MAX_FORWARD_STEP_MPS_PER_CYCLE = 0.06;
  static final double MAX_ROTATION_STEP_RAD_PER_SEC_PER_CYCLE = 0.20;

  public record GroupState(
      Optional<Cameras> activeCamera,
      double noTargetSinceSec,
      double lastForwardMps,
      double lastRotationRadPerSec,
      boolean inGroupMode) {
    public GroupState(Optional<Cameras> activeCamera) {
      this(activeCamera, Double.NaN, 0.0, 0.0, false);
    }
  }

  public record GroupStep(
      GroupState nextState,
      double forwardMps,
      double rotationRadPerSec,
      boolean completed,
      String reason) {}

  private record GroupCameraStats(
      Cameras camera,
      int targetCount,
      double totalArea,
      Optional<PhotonTrackedTarget> representativeTarget) {}

  public static GroupStep fuelPalantirGroup(
      Map<Cameras, Vision.CameraSnapshot> cameraData,
      GroupState currentState,
      FuelPalantir.FuelPalantirMode mode,
      double elapsedSec) {
    Optional<Cameras> activeCamera = currentState.activeCamera();
    double noTargetSinceSec = currentState.noTargetSinceSec();
    boolean inGroupMode = currentState.inGroupMode();

    if (activeCamera.isEmpty()) {
      Optional<GroupCameraStats> acquired = chooseBestGroupCamera(cameraData);
      if (acquired.isPresent()) {
        activeCamera = Optional.of(acquired.get().camera());
      }
    }

    Optional<GroupCameraStats> activeStats =
        activeCamera.map(camera -> summarizeCamera(camera, cameraData.get(camera)));

    if (activeStats.isPresent()) {
      int count = activeStats.get().targetCount();
      if (inGroupMode) {
        if (count <= GROUP_EXIT_TARGET_COUNT) {
          if (Double.isNaN(noTargetSinceSec)) {
            noTargetSinceSec = elapsedSec;
          } else if (elapsedSec - noTargetSinceSec >= GROUP_NO_TARGET_RELEASE_SEC) {
            inGroupMode = false;
            activeCamera = Optional.empty();
            activeStats = Optional.empty();
            noTargetSinceSec = Double.NaN;
          }
        } else {
          noTargetSinceSec = Double.NaN;
        }
      } else if (count >= GROUP_ENTER_TARGET_COUNT) {
        inGroupMode = true;
        noTargetSinceSec = Double.NaN;
      } else {
        activeCamera = Optional.empty();
        activeStats = Optional.empty();
        noTargetSinceSec = Double.NaN;
      }
    }

    double desiredRotation = 0.0;
    double desiredForward = 0.0;
    String reason = "searching_group";

    if (inGroupMode && activeStats.isPresent()) {
      Optional<PhotonTrackedTarget> target = activeStats.get().representativeTarget();
      if (target.isPresent()) {
        double yawDeg = target.get().getYaw();
        if (Math.abs(yawDeg) <= GROUP_TRACK_YAW_DEADBAND_DEG) {
          desiredRotation = 0.0;
        } else {
          desiredRotation =
              MathUtil.clamp(
                  -Math.toRadians(yawDeg) * GROUP_TRACK_ROTATION_GAIN,
                  -GROUP_TRACK_MAX_ROTATION_RAD_PER_SEC,
                  GROUP_TRACK_MAX_ROTATION_RAD_PER_SEC);
        }
        double area = target.get().getArea();
        if (area >= GROUP_APPROACH_STOP_AREA) {
          desiredForward = 0.0;
        } else {
          double areaError = GROUP_APPROACH_STOP_AREA - area;
          desiredForward =
              MathUtil.clamp(
                  areaError * GROUP_APPROACH_AREA_P_GAIN,
                  GROUP_APPROACH_MIN_FORWARD_MPS,
                  GROUP_APPROACH_MAX_FORWARD_MPS);
        }
        reason = "tracking_group";
      } else {
        desiredForward = GROUP_CRUISE_FORWARD_MPS;
        reason = "group_no_representative_target";
      }
    } else {
      desiredRotation = SwerveSubsystem.SEARCH_ROTATION_RAD_PER_SEC;
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
        mode == FuelPalantir.FuelPalantirMode.AUTONOMOUS
            && elapsedSec >= GROUP_AUTONOMOUS_TIMEOUT_SEC;
    if (timedOut) {
      reason = "timeout";
    }

    return new GroupStep(
        new GroupState(activeCamera, noTargetSinceSec, forward, rotation, inGroupMode),
        forward,
        rotation,
        timedOut,
        reason);
  }

  static Optional<PhotonTrackedTarget> getRepresentativeGroupTarget(Vision.CameraSnapshot snapshot) {
    if (snapshot == null
        || snapshot.latestResult() == null
        || !snapshot.latestResult().hasTargets()) {
      return Optional.empty();
    }
    PhotonTrackedTarget best = null;
    double bestArea = Double.NEGATIVE_INFINITY;
    for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
      if (target.getFiducialId() > 0) {
        continue;
      }
      if (target.getArea() > bestArea) {
        bestArea = target.getArea();
        best = target;
      }
    }
    return Optional.ofNullable(best);
  }

  static int countFuelTargets(Vision.CameraSnapshot snapshot) {
    if (snapshot == null
        || snapshot.latestResult() == null
        || !snapshot.latestResult().hasTargets()) {
      return 0;
    }
    int count = 0;
    for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
      if (target.getFiducialId() <= 0) {
        count++;
      }
    }
    return count;
  }

  private static double totalFuelArea(Vision.CameraSnapshot snapshot) {
    if (snapshot == null
        || snapshot.latestResult() == null
        || !snapshot.latestResult().hasTargets()) {
      return 0.0;
    }
    double area = 0.0;
    for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
      if (target.getFiducialId() <= 0) {
        area += target.getArea();
      }
    }
    return area;
  }

  private static GroupCameraStats summarizeCamera(Cameras camera, Vision.CameraSnapshot snapshot) {
    return new GroupCameraStats(
        camera,
        countFuelTargets(snapshot),
        totalFuelArea(snapshot),
        getRepresentativeGroupTarget(snapshot));
  }

  private static Optional<GroupCameraStats> chooseBestGroupCamera(
      Map<Cameras, Vision.CameraSnapshot> cameraData) {
    GroupCameraStats best = null;
    for (Cameras camera : Cameras.values()) {
      GroupCameraStats stats = summarizeCamera(camera, cameraData.get(camera));
      if (stats.targetCount() < GROUP_ENTER_TARGET_COUNT) {
        continue;
      }
      if (best == null
          || stats.targetCount() > best.targetCount()
          || (stats.targetCount() == best.targetCount() && stats.totalArea() > best.totalArea())) {
        best = stats;
      }
    }
    return Optional.ofNullable(best);
  }

  private FuelPalantirGroup() {}
}
