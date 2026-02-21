package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagBallShuttleAuto {
  private static final AtomicInteger AUTO_RUN_COUNTER = new AtomicInteger(0);
  private static final double TAG_APPROACH_DISTANCE_METERS = 0.5;
  private static final double TAG_SEARCH_MAX_RADIANS = 2.0 * Math.PI;
  private static final double SEARCH_ROTATION_RAD_PER_SEC = Math.toRadians(45.0);
  private static final double TAG_CENTER_TOLERANCE_DEG = 3.0;
  private static final double TAG_CENTER_NEAR_TOLERANCE_EXTRA_DEG = 1.0;
  private static final double TAG_CENTER_NEAR_HOLD_SEC = 0.7;
  private static final double APPROACH_MAX_FORWARD_MPS = 0.35;
  private static final double APPROACH_MIN_FORWARD_MPS = 0.35;
  private static final double APPROACH_DISTANCE_TOLERANCE_METERS = 0.20;
  private static final double APPROACH_STALL_DISTANCE_DELTA_METERS = 0.02;
  private static final double APPROACH_STALL_TIME_SEC = 1.8;
  private static final double ANGULAR_TRACKING_GAIN = 2.0;
  private static final double DEBUG_LOG_PERIOD_SEC = 0.25;
  private static final double LOST_TAG_HOLD_SEC = 0.4;
  private static final double LOST_TAG_RECOVERY_ROTATION_RAD_PER_SEC = Math.toRadians(16.0);
  private static final double FUEL_APPROACH_STOP_AREA = 4.5;
  private static final double FUEL_APPROACH_MIN_FORWARD_MPS = 0.10;
  private static final double FUEL_APPROACH_MAX_FORWARD_MPS = 0.35;
  private static final double FUEL_APPROACH_TIMEOUT_SEC = 10.0;
  private static final double DETECTION_CHIRP_TRANSLATION_MPS = 0.22;
  private static final double DETECTION_CHIRP_TIME_SEC = 0.06;
  private static final double DETECTION_CHIRP_GAP_SEC = 0.04;

  private final SwerveSubsystem drivebase;
  private int currentAutoRunId = -1;

  public AprilTagBallShuttleAuto(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
  }

  private record TargetObservation(Cameras camera, PhotonTrackedTarget target) {}

  private void debugAuto(String message) {
    String runId = currentAutoRunId > 0 ? String.format("RUN-%04d", currentAutoRunId) : "RUN-none";
    System.out.printf("[AutoShuttle][%s][%.2f] %s%n", runId, Timer.getFPGATimestamp(), message);
  }

  private boolean shouldDebugLog(AtomicReference<Double> lastLogTimeSec, double periodSec) {
    double now = Timer.getFPGATimestamp();
    if (now - lastLogTimeSec.get() >= periodSec) {
      lastLogTimeSec.set(now);
      return true;
    }
    return false;
  }

  private Command playMotorChirp(int chirpCount, String label) {
    ArrayList<Command> sequence = new ArrayList<>();
    sequence.add(Commands.runOnce(() -> debugAuto("SOUND " + label + " START")));
    for (int i = 0; i < chirpCount; i++) {
      sequence.add(
          Commands.startRun(
                  () -> {},
                  () ->
                      drivebase.drive(
                          new Translation2d(DETECTION_CHIRP_TRANSLATION_MPS, 0), 0, false),
                  drivebase)
              .withTimeout(DETECTION_CHIRP_TIME_SEC));
      sequence.add(Commands.runOnce(() -> drivebase.drive(new Translation2d(0, 0), 0, false)));
      if (i < chirpCount - 1) {
        sequence.add(Commands.waitSeconds(DETECTION_CHIRP_GAP_SEC));
      }
    }
    sequence.add(Commands.runOnce(() -> debugAuto("SOUND " + label + " END")));
    return Commands.sequence(sequence.toArray(new Command[0]));
  }

  private Command playAprilTagFoundSound() {
    return playMotorChirp(1, "APRILTAG");
  }

  private Command playFuelFoundSound() {
    return playMotorChirp(2, "FUEL");
  }

  private Optional<PhotonTrackedTarget> getClosestDetectedObject() {
    var latest = Cameras.CAMERA0.camera.getLatestResult();
    if (!latest.hasTargets()) {
      return Optional.empty();
    }

    PhotonTrackedTarget mostCentered = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : latest.getTargets()) {
      if (target.getFiducialId() > 0) {
        continue;
      }
      double absYaw = Math.abs(target.getYaw());
      if (absYaw < minAbsYaw) {
        minAbsYaw = absYaw;
        mostCentered = target;
      }
    }
    return Optional.ofNullable(mostCentered);
  }

  private Optional<PhotonTrackedTarget> getClosestDetectedObjectAnyCamera() {
    Cameras[] fuelCameras = {Cameras.CAMERA0, Cameras.CAMERA1};
    PhotonTrackedTarget mostCentered = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (Cameras camera : fuelCameras) {
      var latest = camera.camera.getLatestResult();
      if (!latest.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : latest.getTargets()) {
        if (target.getFiducialId() > 0) {
          continue;
        }
        double absYaw = Math.abs(target.getYaw());
        if (absYaw < minAbsYaw) {
          minAbsYaw = absYaw;
          mostCentered = target;
        }
      }
    }
    return Optional.ofNullable(mostCentered);
  }

  private Optional<TargetObservation> getClosestDetectedObjectObservationAnyCamera() {
    Cameras[] fuelCameras = {Cameras.CAMERA0, Cameras.CAMERA1};
    TargetObservation mostCentered = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (Cameras camera : fuelCameras) {
      var latest = camera.camera.getLatestResult();
      if (!latest.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : latest.getTargets()) {
        if (target.getFiducialId() > 0) {
          continue;
        }
        double absYaw = Math.abs(target.getYaw());
        if (absYaw < minAbsYaw) {
          minAbsYaw = absYaw;
          mostCentered = new TargetObservation(camera, target);
        }
      }
    }
    return Optional.ofNullable(mostCentered);
  }

  private double getTargetPlanarDistanceMeters(PhotonTrackedTarget target) {
    var translation = target.getBestCameraToTarget().getTranslation();
    return translation.getNorm();
  }

  private Optional<TargetObservation> getClosestVisibleAprilTagObservation(int excludedTagId) {
    TargetObservation closest = null;
    double closestDistance = Double.POSITIVE_INFINITY;
    Cameras[] tagCameras = {Cameras.CAMERA0, Cameras.CAMERA1};

    for (Cameras camera : tagCameras) {
      var latest = camera.camera.getLatestResult();
      if (!latest.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : latest.getTargets()) {
        int fiducialId = target.getFiducialId();
        if (fiducialId <= 0 || fiducialId == excludedTagId) {
          continue;
        }
        double planarDistance = getTargetPlanarDistanceMeters(target);
        if (planarDistance < closestDistance) {
          closestDistance = planarDistance;
          closest = new TargetObservation(camera, target);
        }
      }
    }
    return Optional.ofNullable(closest);
  }

  private Optional<TargetObservation> getVisibleAprilTagById(int tagId) {
    if (tagId <= 0) {
      return Optional.empty();
    }
    TargetObservation closest = null;
    double closestDistance = Double.POSITIVE_INFINITY;
    Cameras[] tagCameras = {Cameras.CAMERA0, Cameras.CAMERA1};

    for (Cameras camera : tagCameras) {
      var latest = camera.camera.getLatestResult();
      if (!latest.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : latest.getTargets()) {
        if (target.getFiducialId() != tagId) {
          continue;
        }
        double planarDistance = getTargetPlanarDistanceMeters(target);
        if (planarDistance < closestDistance) {
          closestDistance = planarDistance;
          closest = new TargetObservation(camera, target);
        }
      }
    }
    return Optional.ofNullable(closest);
  }

  private Optional<TargetObservation> getVisibleAprilTagByIdOnCamera(int tagId, Cameras camera) {
    if (tagId <= 0) {
      return Optional.empty();
    }
    var latest = camera.camera.getLatestResult();
    if (!latest.hasTargets()) {
      return Optional.empty();
    }

    PhotonTrackedTarget closest = null;
    double closestDistance = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : latest.getTargets()) {
      if (target.getFiducialId() != tagId) {
        continue;
      }
      double planarDistance = getTargetPlanarDistanceMeters(target);
      if (planarDistance < closestDistance) {
        closestDistance = planarDistance;
        closest = target;
      }
    }

    return closest == null ? Optional.empty() : Optional.of(new TargetObservation(camera, closest));
  }

  private String buildTagVisibilitySummary(int excludedTagId) {
    Cameras[] cameras = {Cameras.CAMERA0, Cameras.CAMERA1};
    StringBuilder sb = new StringBuilder();
    for (Cameras cam : cameras) {
      var latest = cam.camera.getLatestResult();
      int count = 0;
      int chosenId = -1;
      if (latest.hasTargets()) {
        for (PhotonTrackedTarget t : latest.getTargets()) {
          int id = t.getFiducialId();
          if (id > 0 && id != excludedTagId) {
            count++;
            chosenId = id;
          }
        }
      }
      if (sb.length() > 0) {
        sb.append(" | ");
      }
      sb.append(cam.name()).append(":tags=").append(count);
      if (chosenId > 0) {
        sb.append(" lastId=").append(chosenId);
      }
    }
    return sb.toString();
  }

  private double calculateRotationFromYawDeg(double yawDeg) {
    return MathUtil.clamp(
        -Math.toRadians(yawDeg) * ANGULAR_TRACKING_GAIN,
        -SEARCH_ROTATION_RAD_PER_SEC,
        SEARCH_ROTATION_RAD_PER_SEC);
  }

  private Command alignToTargetWithRotationLimit(
      java.util.function.Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
      double centeredToleranceDeg,
      double maxRotationRadians) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Boolean> centered = new AtomicReference<>(false);
    AtomicReference<Double> nearCenteredSinceSec = new AtomicReference<>(Double.NaN);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return Commands.startRun(
            () -> {
              lastHeadingRad.set(drivebase.getHeading().getRadians());
              rotatedRad.set(0.0);
              centered.set(false);
              nearCenteredSinceSec.set(Double.NaN);
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              debugAuto(
                  String.format(
                      "ALIGN START tol=%.1fdeg maxRot=%.1fdeg",
                      centeredToleranceDeg, Math.toDegrees(maxRotationRadians)));
            },
            () -> {
              double currentHeading = drivebase.getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);

              Optional<PhotonTrackedTarget> target = targetSupplier.get();
              if (target.isPresent()) {
                double yawDeg = target.get().getYaw();
                double absYaw = Math.abs(yawDeg);
                boolean hardCentered = absYaw <= centeredToleranceDeg;
                boolean nearCentered =
                    absYaw <= centeredToleranceDeg + TAG_CENTER_NEAR_TOLERANCE_EXTRA_DEG;

                if (hardCentered) {
                  centered.set(true);
                  nearCenteredSinceSec.set(Double.NaN);
                } else if (nearCentered) {
                  if (Double.isNaN(nearCenteredSinceSec.get())) {
                    nearCenteredSinceSec.set(Timer.getFPGATimestamp());
                  }
                  centered.set(
                      Timer.getFPGATimestamp() - nearCenteredSinceSec.get()
                          >= TAG_CENTER_NEAR_HOLD_SEC);
                } else {
                  nearCenteredSinceSec.set(Double.NaN);
                  centered.set(false);
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "ALIGN tracking yaw=%.2fdeg centered=%s nearSince=%s rotated=%.1fdeg",
                          yawDeg,
                          centered.get(),
                          Double.isNaN(nearCenteredSinceSec.get())
                              ? "n/a"
                              : String.format(
                                  "%.2f", Timer.getFPGATimestamp() - nearCenteredSinceSec.get()),
                          Math.toDegrees(rotatedRad.get())));
                }
                drivebase.drive(
                    new Translation2d(0, 0), calculateRotationFromYawDeg(yawDeg), false);
              } else {
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "ALIGN no target, spinning rotated=%.1fdeg",
                          Math.toDegrees(rotatedRad.get())));
                }
                drivebase.drive(new Translation2d(0, 0), SEARCH_ROTATION_RAD_PER_SEC, false);
              }
            },
            drivebase)
        .until(() -> centered.get() || rotatedRad.get() >= maxRotationRadians)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "ALIGN END centered=%s rotated=%.1fdeg",
                      centered.get(), Math.toDegrees(rotatedRad.get())));
              drivebase.drive(new Translation2d(0, 0), 0, false);
            });
  }

  private Command approachKnownTagByVision(AtomicInteger tagIdRef, double distanceMeters) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Boolean> reached = new AtomicReference<>(false);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);
    AtomicReference<Double> lastSeenTargetTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);
    AtomicReference<Double> lastSeenYawDeg = new AtomicReference<>(0.0);
    AtomicReference<Double> lastDistanceMeters = new AtomicReference<>(Double.NaN);
    AtomicReference<Double> lastProgressTimeSec = new AtomicReference<>(0.0);
    AtomicReference<Cameras> lockedCamera = new AtomicReference<>(null);

    return Commands.startRun(
            () -> {
              lastHeadingRad.set(drivebase.getHeading().getRadians());
              rotatedRad.set(0.0);
              reached.set(false);
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              lastSeenTargetTimeSec.set(Double.NEGATIVE_INFINITY);
              lastSeenYawDeg.set(0.0);
              lastDistanceMeters.set(Double.NaN);
              lastProgressTimeSec.set(Timer.getFPGATimestamp());
              lockedCamera.set(null);
              debugAuto(
                  String.format(
                      "APPROACH START tagId=%d distanceGoal=%.2fm",
                      tagIdRef.get(), distanceMeters));
            },
            () -> {
              double currentHeading = drivebase.getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);

              Optional<TargetObservation> observation =
                  lockedCamera.get() == null
                      ? getVisibleAprilTagById(tagIdRef.get())
                      : getVisibleAprilTagByIdOnCamera(tagIdRef.get(), lockedCamera.get());
              if (observation.isPresent()) {
                lockedCamera.set(observation.get().camera());
              } else if (lockedCamera.get() != null) {
                observation = getVisibleAprilTagById(tagIdRef.get());
                observation.ifPresent(o -> lockedCamera.set(o.camera()));
              }

              if (observation.isEmpty()) {
                double timeSinceSeen = Timer.getFPGATimestamp() - lastSeenTargetTimeSec.get();
                double recoveryRotation = calculateRotationFromYawDeg(lastSeenYawDeg.get());
                if (timeSinceSeen <= LOST_TAG_HOLD_SEC) {
                  if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                    debugAuto(
                        String.format(
                            "APPROACH tagId=%d briefly lost (%.2fs), holding heading lastYaw=%.2fdeg",
                            tagIdRef.get(), timeSinceSeen, lastSeenYawDeg.get()));
                  }
                  drivebase.drive(new Translation2d(0, 0), recoveryRotation, false);
                  return;
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "APPROACH tagId=%d not visible, spinning rotated=%.1fdeg",
                          tagIdRef.get(), Math.toDegrees(rotatedRad.get())));
                }
                drivebase.drive(
                    new Translation2d(0, 0), LOST_TAG_RECOVERY_ROTATION_RAD_PER_SEC, false);
                return;
              }

              PhotonTrackedTarget target = observation.get().target();
              double distance = getTargetPlanarDistanceMeters(target);
              double yawDeg = target.getYaw();
              lastSeenTargetTimeSec.set(Timer.getFPGATimestamp());
              lastSeenYawDeg.set(yawDeg);
              if (Double.isNaN(lastDistanceMeters.get())
                  || Math.abs(lastDistanceMeters.get() - distance)
                      > APPROACH_STALL_DISTANCE_DELTA_METERS) {
                lastProgressTimeSec.set(Timer.getFPGATimestamp());
              }
              lastDistanceMeters.set(distance);
              double rotation = calculateRotationFromYawDeg(yawDeg);
              double distanceError = distance - distanceMeters;
              double forwardSpeed = 0.0;
              if (distanceError > 0.0) {
                forwardSpeed =
                    MathUtil.clamp(
                        distanceError * 0.9, APPROACH_MIN_FORWARD_MPS, APPROACH_MAX_FORWARD_MPS);
              }
              double cameraForwardSign = observation.get().camera() == Cameras.CAMERA1 ? -1.0 : 1.0;
              double commandedForward = cameraForwardSign * forwardSpeed;
              drivebase.drive(new Translation2d(commandedForward, 0), rotation, false);
              boolean noProgressLongEnough =
                  (Timer.getFPGATimestamp() - lastProgressTimeSec.get()) >= APPROACH_STALL_TIME_SEC;
              reached.set(
                  (distance <= (distanceMeters + APPROACH_DISTANCE_TOLERANCE_METERS)
                          && Math.abs(yawDeg) <= TAG_CENTER_TOLERANCE_DEG)
                      || (noProgressLongEnough
                          && distance <= (distanceMeters + APPROACH_DISTANCE_TOLERANCE_METERS)
                          && Math.abs(yawDeg) <= (TAG_CENTER_TOLERANCE_DEG + 1.0)));
              if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                debugAuto(
                    String.format(
                        "APPROACH tagId=%d camera=%s dist=%.2fm yaw=%.2fdeg fwd=%.2f rot=%.2f reached=%s stall=%.2fs",
                        tagIdRef.get(),
                        observation.get().camera().name(),
                        distance,
                        yawDeg,
                        commandedForward,
                        rotation,
                        reached.get(),
                        Timer.getFPGATimestamp() - lastProgressTimeSec.get()));
              }
            },
            drivebase)
        .until(() -> reached.get() || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "APPROACH END tagId=%d reached=%s rotated=%.1fdeg",
                      tagIdRef.get(), reached.get(), Math.toDegrees(rotatedRad.get())));
              drivebase.drive(new Translation2d(0, 0), 0, false);
            });
  }

  private Command updateTagIdFromVisibleTarget(
      AtomicInteger tagToUpdate, IntSupplier excludedTagId) {
    return Commands.runOnce(
        () -> {
          getClosestVisibleAprilTagObservation(excludedTagId.getAsInt())
              .ifPresent(observation -> tagToUpdate.set(observation.target().getFiducialId()));
          debugAuto(
              String.format(
                  "TAG CANDIDATE scan exclude=%d selected=%d",
                  excludedTagId.getAsInt(), tagToUpdate.get()));
        });
  }

  private Command scanForTagWith1080Limit(AtomicInteger targetTagId, IntSupplier excludedTagId) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetTagId.set(-1);
              debugAuto(
                  String.format(
                      "TAG SEARCH START exclude=%d max=%.1fdeg",
                      excludedTagId.getAsInt(), Math.toDegrees(TAG_SEARCH_MAX_RADIANS)));
            }),
        updateTagIdFromVisibleTarget(targetTagId, excludedTagId),
        Commands.either(
            Commands.none(),
            Commands.startRun(
                    () -> {
                      lastHeadingRad.set(drivebase.getHeading().getRadians());
                      rotatedRad.set(0.0);
                      lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
                    },
                    () -> {
                      double currentHeading = drivebase.getHeading().getRadians();
                      double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
                      rotatedRad.set(rotatedRad.get() + Math.abs(delta));
                      lastHeadingRad.set(currentHeading);
                      drivebase.drive(new Translation2d(0, 0), SEARCH_ROTATION_RAD_PER_SEC, false);
                      Optional<TargetObservation> candidate =
                          getClosestVisibleAprilTagObservation(excludedTagId.getAsInt());
                      candidate.ifPresent(
                          observation -> targetTagId.set(observation.target().getFiducialId()));
                      if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                        if (candidate.isPresent()) {
                          PhotonTrackedTarget t = candidate.get().target();
                          debugAuto(
                              String.format(
                                  "TAG SEARCH tracking id=%d camera=%s yaw=%.2fdeg dist=%.2fm rotated=%.1fdeg",
                                  t.getFiducialId(),
                                  candidate.get().camera().name(),
                                  t.getYaw(),
                                  getTargetPlanarDistanceMeters(t),
                                  Math.toDegrees(rotatedRad.get())));
                        } else {
                          debugAuto(
                              String.format(
                                  "TAG SEARCH no tag rotated=%.1fdeg [%s]",
                                  Math.toDegrees(rotatedRad.get()),
                                  buildTagVisibilitySummary(excludedTagId.getAsInt())));
                        }
                      }
                    },
                    drivebase)
                .until(() -> targetTagId.get() > 0 || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS)
                .finallyDo(
                    () -> {
                      debugAuto(
                          String.format(
                              "TAG SEARCH END selected=%d rotated=%.1fdeg",
                              targetTagId.get(), Math.toDegrees(rotatedRad.get())));
                      drivebase.drive(new Translation2d(0, 0), 0, false);
                    }),
            () -> targetTagId.get() > 0 && targetTagId.get() != excludedTagId.getAsInt()),
        Commands.either(playAprilTagFoundSound(), Commands.none(), () -> targetTagId.get() > 0));
  }

  private Command rotateRelativeDegrees(double degrees) {
    double targetRadians = Math.abs(Math.toRadians(degrees));
    double direction = Math.signum(degrees) == 0.0 ? 1.0 : Math.signum(degrees);
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);

    return Commands.startRun(
            () -> {
              lastHeadingRad.set(drivebase.getHeading().getRadians());
              rotatedRad.set(0.0);
              debugAuto(String.format("ROTATE START degrees=%.1f", degrees));
            },
            () -> {
              double currentHeading = drivebase.getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);
              drivebase.drive(
                  new Translation2d(0, 0), direction * SEARCH_ROTATION_RAD_PER_SEC, false);
            },
            drivebase)
        .until(() -> rotatedRad.get() >= targetRadians)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "ROTATE END target=%.1f actual=%.1f",
                      degrees, Math.toDegrees(rotatedRad.get())));
              drivebase.drive(new Translation2d(0, 0), 0, false);
            });
  }

  private Command findSecondTagFromCurrentTag(
      AtomicInteger currentTagId, AtomicInteger secondTagId) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                debugAuto(
                    String.format(
                        "SECOND TAG PLAN: center current=%d, rotate 180, spin search",
                        currentTagId.get()))),
        centerOnKnownTag(currentTagId),
        rotateRelativeDegrees(180.0),
        scanForTagWith1080Limit(secondTagId, currentTagId::get));
  }

  private Command centerOnKnownTag(AtomicInteger tagIdRef) {
    return Commands.sequence(
        Commands.runOnce(
            () -> debugAuto(String.format("CENTER TAG START tagId=%d", tagIdRef.get()))),
        Commands.either(
            alignToTargetWithRotationLimit(
                () -> getVisibleAprilTagById(tagIdRef.get()).map(TargetObservation::target),
                TAG_CENTER_TOLERANCE_DEG,
                TAG_SEARCH_MAX_RADIANS),
            Commands.none(),
            () -> tagIdRef.get() > 0),
        Commands.runOnce(
            () -> debugAuto(String.format("CENTER TAG END tagId=%d", tagIdRef.get()))));
  }

  private Command findAndApproachFuelWithAnyCamera(AtomicBoolean fuelFound) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);
    AtomicReference<Double> startTimeSec = new AtomicReference<>(0.0);
    AtomicReference<Cameras> lockedFuelCamera = new AtomicReference<>(null);

    return Commands.startRun(
            () -> {
              fuelFound.set(false);
              lastHeadingRad.set(drivebase.getHeading().getRadians());
              rotatedRad.set(0.0);
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              startTimeSec.set(Timer.getFPGATimestamp());
              lockedFuelCamera.set(null);
              debugAuto(
                  String.format(
                      "FUEL ACQUIRE START fuelSeenAny=%s fuelSeenFront=%s max=%.1fdeg",
                      getClosestDetectedObjectAnyCamera().isPresent(),
                      getClosestDetectedObject().isPresent(),
                      Math.toDegrees(TAG_SEARCH_MAX_RADIANS)));
            },
            () -> {
              double currentHeading = drivebase.getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);

              if (lockedFuelCamera.get() == null) {
                Optional<TargetObservation> firstSeen =
                    getClosestDetectedObjectObservationAnyCamera();
                if (firstSeen.isPresent()) {
                  lockedFuelCamera.set(firstSeen.get().camera());
                  debugAuto(
                      String.format(
                          "FUEL LOCK firstSeen camera=%s yaw=%.2f area=%.3f",
                          firstSeen.get().camera().name(),
                          firstSeen.get().target().getYaw(),
                          firstSeen.get().target().getArea()));
                }
              }

              Optional<PhotonTrackedTarget> fuelFront = getClosestDetectedObject();
              boolean canApproachFront =
                  fuelFront.isPresent()
                      && (lockedFuelCamera.get() == Cameras.CAMERA0
                          || lockedFuelCamera.get() == Cameras.CAMERA1);
              if (canApproachFront) {
                double yawDeg = fuelFront.get().getYaw();
                double rotation = calculateRotationFromYawDeg(yawDeg);
                double area = fuelFront.get().getArea();
                double forward = 0.0;
                if (area >= FUEL_APPROACH_STOP_AREA) {
                  fuelFound.set(true);
                } else {
                  double areaError = FUEL_APPROACH_STOP_AREA - area;
                  forward =
                      MathUtil.clamp(
                          areaError * 0.08,
                          FUEL_APPROACH_MIN_FORWARD_MPS,
                          FUEL_APPROACH_MAX_FORWARD_MPS);
                }
                drivebase.drive(new Translation2d(forward, 0), rotation, false);
                if (forward > 0.0 && shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL DRIVE front area=%.3f yaw=%.2fdeg fwd=%.2f rot=%.2f",
                          area, yawDeg, forward, rotation));
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL ACQUIRE front yaw=%.2fdeg area=%.3f fwd=%.2f centered=%s reached=%s rotated=%.1fdeg",
                          yawDeg,
                          area,
                          forward,
                          true,
                          fuelFound.get(),
                          Math.toDegrees(rotatedRad.get())));
                }
              } else {
                double spin = SEARCH_ROTATION_RAD_PER_SEC;
                drivebase.drive(new Translation2d(0, 0), spin, false);
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL ACQUIRE no front fuel rotated=%.1fdeg lockedCamera=%s",
                          Math.toDegrees(rotatedRad.get()),
                          lockedFuelCamera.get() == null ? "none" : lockedFuelCamera.get().name()));
                }
              }
            },
            drivebase)
        .until(
            () ->
                fuelFound.get()
                    || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS
                    || (Timer.getFPGATimestamp() - startTimeSec.get()) >= FUEL_APPROACH_TIMEOUT_SEC)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "FUEL ACQUIRE END fuelFound=%s rotated=%.1fdeg elapsed=%.2fs",
                      fuelFound.get(),
                      Math.toDegrees(rotatedRad.get()),
                      Timer.getFPGATimestamp() - startTimeSec.get()));
              drivebase.drive(new Translation2d(0, 0), 0, false);
            });
  }

  private Command findFuelForKnownTag(AtomicInteger activeTagId, AtomicBoolean fuelFound) {
    return Commands.sequence(
        Commands.runOnce(
            () -> debugAuto(String.format("FUEL ROUTINE START tagId=%d", activeTagId.get()))),
        centerOnKnownTag(activeTagId),
        findAndApproachFuelWithAnyCamera(fuelFound),
        Commands.either(playFuelFoundSound(), Commands.none(), fuelFound::get),
        Commands.runOnce(
            () -> {
              if (!fuelFound.get()) {
                debugAuto(
                    String.format(
                        "FUEL ROUTINE no fuel found after spin search tagId=%d, continuing",
                        activeTagId.get()));
              }
            }),
        Commands.runOnce(
            () ->
                debugAuto(
                    String.format(
                        "FUEL ROUTINE END tagId=%d fuelFound=%s",
                        activeTagId.get(), fuelFound.get()))));
  }

  public Command build(int repetitionCount) {
    int cycles = Math.max(1, repetitionCount);
    AtomicInteger currentTagId = new AtomicInteger(-1);
    AtomicInteger nextTagId = new AtomicInteger(-1);
    AtomicBoolean keepRunning = new AtomicBoolean(true);
    AtomicBoolean fuelFoundAtCurrentTag = new AtomicBoolean(false);

    ArrayList<Command> loopCommands = new ArrayList<>();
    for (int i = 0; i < cycles; i++) {
      final int cycleIndex = i + 1;
      loopCommands.add(
          Commands.either(
              Commands.sequence(
                  Commands.runOnce(
                      () -> debugAuto(String.format("CYCLE %d/%d START", cycleIndex, cycles))),
                  centerOnKnownTag(currentTagId),
                  approachKnownTagByVision(currentTagId, TAG_APPROACH_DISTANCE_METERS),
                  findFuelForKnownTag(currentTagId, fuelFoundAtCurrentTag),
                  findSecondTagFromCurrentTag(currentTagId, nextTagId),
                  Commands.runOnce(
                      () -> {
                        if (nextTagId.get() > 0) {
                          debugAuto(
                              String.format(
                                  "NEXT TAG FOUND cycle=%d id=%d (fuelFound=%s)",
                                  cycleIndex, nextTagId.get(), fuelFoundAtCurrentTag.get()));
                          currentTagId.set(nextTagId.get());
                        } else {
                          debugAuto(
                              String.format(
                                  "NEXT TAG NOT FOUND cycle=%d; stopping cycle (fuelFound=%s)",
                                  cycleIndex, fuelFoundAtCurrentTag.get()));
                          keepRunning.set(false);
                        }
                      }),
                  Commands.runOnce(
                      () -> debugAuto(String.format("CYCLE %d/%d END", cycleIndex, cycles)))),
              Commands.none(),
              keepRunning::get));
    }

    return Commands.sequence(
        Commands.runOnce(() -> currentAutoRunId = AUTO_RUN_COUNTER.incrementAndGet()),
        Commands.runOnce(
            () -> {
              currentTagId.set(-1);
              nextTagId.set(-1);
              keepRunning.set(true);
              fuelFoundAtCurrentTag.set(false);
            }),
        Commands.runOnce(() -> debugAuto(String.format("AUTO START cycles=%d", cycles))),
        scanForTagWith1080Limit(currentTagId, () -> -1),
        Commands.either(
            Commands.none(),
            Commands.runOnce(
                () -> {
                  keepRunning.set(false);
                  debugAuto(
                      String.format(
                          "INITIAL TAG NOT FOUND after %.1fdeg; ending auto",
                          Math.toDegrees(TAG_SEARCH_MAX_RADIANS)));
                }),
            () -> currentTagId.get() > 0),
        Commands.sequence(loopCommands.toArray(new Command[0])),
        Commands.runOnce(
            () ->
                debugAuto(
                    String.format(
                        "AUTO END currentTag=%d keepRunning=%s",
                        currentTagId.get(), keepRunning.get()))));
  }
}
