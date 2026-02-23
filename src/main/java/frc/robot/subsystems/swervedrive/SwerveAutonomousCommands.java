package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BufferedLogger;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

final class SwerveAutonomousCommands {
  private static final AtomicInteger AUTO_RUN_COUNTER = new AtomicInteger(0);
  private static final double TAG_APPROACH_DISTANCE_METERS = 0.5;
  private static final double TAG_SEARCH_MAX_RADIANS = 2.0 * Math.PI; // 360 deg
  private static final double SEARCH_ROTATION_RAD_PER_SEC = Math.toRadians(45.0);
  private static final double TAG_CENTER_TOLERANCE_DEG = 3.0;
  private static final double TAG_CENTER_NEAR_TOLERANCE_EXTRA_DEG = 1.0;
  private static final double TAG_CENTER_NEAR_HOLD_SEC = 0.7;
  private static final double BALL_CENTER_TOLERANCE_DEG = 8.0;
  private static final double APPROACH_MAX_FORWARD_MPS = 0.35;
  private static final double APPROACH_MIN_FORWARD_MPS = 0.10;
  private static final double APPROACH_DISTANCE_TOLERANCE_METERS = 0.20;
  private static final double APPROACH_STALL_DISTANCE_DELTA_METERS = 0.02;
  private static final double APPROACH_STALL_TIME_SEC = 1.8;
  private static final double ANGULAR_TRACKING_GAIN = 2.0;
  private static final double DEBUG_LOG_PERIOD_SEC = 0.25;
  private static final double LOST_TAG_HOLD_SEC = 0.4;
  private static final double LOST_TAG_RECOVERY_ROTATION_RAD_PER_SEC = Math.toRadians(16.0);
  private static final double DETECTION_CHIRP_TRANSLATION_MPS = 0.22;
  private static final double DETECTION_CHIRP_TIME_SEC = 0.06;
  private static final double DETECTION_CHIRP_GAP_SEC = 0.04;

  private final SwerveSubsystem subsystem;
  private final SwerveDrive swerveDrive;
  private final Supplier<Vision> visionSupplier;

  private int currentAutoRunId = -1;

  SwerveAutonomousCommands(
      SwerveSubsystem subsystem, SwerveDrive swerveDrive, Supplier<Vision> visionSupplier) {
    this.subsystem = subsystem;
    this.swerveDrive = swerveDrive;
    this.visionSupplier = visionSupplier;
  }

  Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  Command driveToPose(Pose2d pose) {
    PathConstraints constraints =
        new PathConstraints(
            0.35,
            0.35,
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
  }

  public Command fuelPalantirCommand(FuelPalantir.FuelPalantirMode mode) {
    return Commands.defer(
            () -> {
              AtomicReference<Double> startTimeSec = new AtomicReference<>(0.0);
              AtomicReference<FuelPalantir.FuelPalantirState> state =
                  new AtomicReference<>(new FuelPalantir.FuelPalantirState(Optional.empty()));
              AtomicReference<FuelPalantir.FuelPalantirStep> lastStep = new AtomicReference<>(null);
              AtomicReference<Double> lastStatusLogTimeSec =
                  new AtomicReference<>(Double.NEGATIVE_INFINITY);

              return subsystem
                  .startRun(
                      () -> {
                        currentAutoRunId = AUTO_RUN_COUNTER.incrementAndGet();
                        startTimeSec.set(Timer.getFPGATimestamp());
                        state.set(new FuelPalantir.FuelPalantirState(Optional.empty()));
                        lastStep.set(null);
                        debugAuto(String.format("FUEL PALANTIR START mode=%s", mode));
                      },
                      () -> {
                        Vision vision = visionSupplier.get();
                        if (vision == null) {
                          debugAuto("FUEL PALANTIR no vision instance available");
                          lastStep.set(
                              new FuelPalantir.FuelPalantirStep(
                                  state.get(), 0.0, 0.0, true, "vision_not_initialized"));
                          swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
                          return;
                        }
                        double elapsed = Timer.getFPGATimestamp() - startTimeSec.get();
                        Map<Cameras, Vision.CameraSnapshot> cameraData = vision.getLatestCameraData();
                        FuelPalantir.FuelPalantirStep step =
                            FuelPalantir.fuelPalantir(cameraData, state.get(), mode, elapsed);
                        state.set(step.nextState());
                        lastStep.set(step);
                        Optional<Cameras> lockedCameraForDrive = step.nextState().lockedCamera();
                        double commandedForwardMps = step.forwardMps();
                        if (lockedCameraForDrive.isPresent()
                            && lockedCameraForDrive.get() == Cameras.BACK_CAMERA) {
                          commandedForwardMps = -commandedForwardMps;
                        }
                        swerveDrive.drive(
                            new Translation2d(commandedForwardMps, 0),
                            step.rotationRadPerSec(),
                            false,
                            false);
                        if (step.completed() || shouldDebugLog(lastStatusLogTimeSec, 1.0)) {
                          Optional<PhotonTrackedTarget> backCameraTarget =
                              FuelPalantir.getClosestNonFiducialTarget(
                                  cameraData.get(Cameras.BACK_CAMERA));
                          Optional<PhotonTrackedTarget> frontCameraTarget =
                              FuelPalantir.getClosestNonFiducialTarget(
                                  cameraData.get(Cameras.FRONT_CAMERA));
                          Optional<Cameras> lockedCamera = step.nextState().lockedCamera();
                          Optional<PhotonTrackedTarget> lockedTarget =
                              lockedCamera.flatMap(
                                  camera ->
                                      FuelPalantir.getClosestNonFiducialTarget(
                                          cameraData.get(camera)));

                          debugAuto(
                              String.format(
                                  "FUEL PALANTIR STATUS mode=%s elapsed=%.2fs"
                                      + " locked=%s backTarget=%s frontTarget=%s"
                                      + " lockedYaw=%.1f lockedArea=%.2f"
                                      + " fwd=%.2f rot=%.2f reason=%s",
                                  mode,
                                  elapsed,
                                  lockedCamera.map(Enum::name).orElse("none"),
                                  backCameraTarget.isPresent(),
                                  frontCameraTarget.isPresent(),
                                  lockedTarget.map(PhotonTrackedTarget::getYaw).orElse(Double.NaN),
                                  lockedTarget.map(PhotonTrackedTarget::getArea).orElse(Double.NaN),
                                  commandedForwardMps,
                                  step.rotationRadPerSec(),
                                  step.reason()));
                        }
                      })
                  .until(() -> lastStep.get() != null && lastStep.get().completed())
                  .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false))
                  .andThen(
                      subsystem
                          .runOnce(
                              () -> {
                                FuelPalantir.FuelPalantirStep step = lastStep.get();
                                String reason = step == null ? "unknown" : step.reason();
                                debugAuto(
                                    String.format(
                                        "FUEL PALANTIR END mode=%s elapsed=%.2fs reason=%s",
                                        mode,
                                        Timer.getFPGATimestamp() - startTimeSec.get(),
                                        reason));
                                currentAutoRunId = -1;
                              }))
                  .withName("FuelPalantirCommand-" + mode.name());
            },
            Set.of(subsystem))
        .withName("FuelPalantirCommand-" + mode.name());
  }

  private record TargetObservation(Cameras camera, PhotonTrackedTarget target) {}

  private void debugAuto(String message) {
    String runId = currentAutoRunId > 0 ? String.format("RUN-%04d", currentAutoRunId) : "RUN-none";
    BufferedLogger.getInstance().printf("[AutoShuttle][%s] %s", runId, message);
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
          subsystem
              .startRun(
                  () -> {},
                  () ->
                      swerveDrive.drive(
                          new Translation2d(DETECTION_CHIRP_TRANSLATION_MPS, 0), 0, false, false))
              .withTimeout(DETECTION_CHIRP_TIME_SEC));
      sequence.add(
          Commands.runOnce(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false)));
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

  private double getTargetPlanarDistanceMeters(PhotonTrackedTarget target) {
    var translation = target.getBestCameraToTarget().getTranslation();
    return translation.getNorm();
  }

  private Optional<TargetObservation> getClosestVisibleAprilTagObservation(int excludedTagId) {
    TargetObservation closest = null;
    double closestDistance = Double.POSITIVE_INFINITY;
    Cameras[] tagCameras = {Cameras.BACK_CAMERA, Cameras.FRONT_CAMERA};

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
    Cameras[] tagCameras = {Cameras.BACK_CAMERA, Cameras.FRONT_CAMERA};

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
    Cameras[] cameras = {Cameras.BACK_CAMERA, Cameras.FRONT_CAMERA};
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

  private static double calculateRotationFromYawDeg(double yawDeg) {
    return MathUtil.clamp(
        -Math.toRadians(yawDeg) * ANGULAR_TRACKING_GAIN,
        -SEARCH_ROTATION_RAD_PER_SEC,
        SEARCH_ROTATION_RAD_PER_SEC);
  }

  private Command alignToTargetWithRotationLimit(
      Supplier<Optional<PhotonTrackedTarget>> targetSupplier,
      double centeredToleranceDeg,
      double maxRotationRadians) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Boolean> centered = new AtomicReference<>(false);
    AtomicReference<Double> nearCenteredSinceSec = new AtomicReference<>(Double.NaN);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return subsystem
        .startRun(
            () -> {
              lastHeadingRad.set(subsystem.getHeading().getRadians());
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
              double currentHeading = subsystem.getHeading().getRadians();
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
                swerveDrive.drive(
                    new Translation2d(0, 0), calculateRotationFromYawDeg(yawDeg), false, false);
              } else {
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "ALIGN no target, spinning rotated=%.1fdeg",
                          Math.toDegrees(rotatedRad.get())));
                }
                swerveDrive.drive(
                    new Translation2d(0, 0), SEARCH_ROTATION_RAD_PER_SEC, false, false);
              }
            })
        .until(() -> centered.get() || rotatedRad.get() >= maxRotationRadians)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "ALIGN END centered=%s rotated=%.1fdeg",
                      centered.get(), Math.toDegrees(rotatedRad.get())));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
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

    return subsystem
        .startRun(
            () -> {
              lastHeadingRad.set(subsystem.getHeading().getRadians());
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
              double currentHeading = subsystem.getHeading().getRadians();
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
                  swerveDrive.drive(new Translation2d(0, 0), recoveryRotation, false, false);
                  return;
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "APPROACH tagId=%d not visible, spinning rotated=%.1fdeg",
                          tagIdRef.get(), Math.toDegrees(rotatedRad.get())));
                }
                swerveDrive.drive(
                    new Translation2d(0, 0), LOST_TAG_RECOVERY_ROTATION_RAD_PER_SEC, false, false);
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
              double cameraForwardSign =
                  observation.get().camera() == Cameras.FRONT_CAMERA ? 1.0 : -1.0;
              double commandedForward = cameraForwardSign * forwardSpeed;
              swerveDrive.drive(new Translation2d(commandedForward, 0), rotation, false, false);
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
            })
        .until(() -> reached.get() || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "APPROACH END tagId=%d reached=%s rotated=%.1fdeg",
                      tagIdRef.get(), reached.get(), Math.toDegrees(rotatedRad.get())));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command updateTagIdFromVisibleTarget(AtomicInteger tagToUpdate, IntSupplier excludedTagId) {
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
            subsystem
                .startRun(
                    () -> {
                      lastHeadingRad.set(subsystem.getHeading().getRadians());
                      rotatedRad.set(0.0);
                      lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
                    },
                    () -> {
                      double currentHeading = subsystem.getHeading().getRadians();
                      double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
                      rotatedRad.set(rotatedRad.get() + Math.abs(delta));
                      lastHeadingRad.set(currentHeading);
                      swerveDrive.drive(
                          new Translation2d(0, 0), SEARCH_ROTATION_RAD_PER_SEC, false, false);
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
                    })
                .until(() -> targetTagId.get() > 0 || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS)
                .finallyDo(
                    () -> {
                      debugAuto(
                          String.format(
                              "TAG SEARCH END selected=%d rotated=%.1fdeg",
                              targetTagId.get(), Math.toDegrees(rotatedRad.get())));
                      swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
                    }),
            () -> targetTagId.get() > 0 && targetTagId.get() != excludedTagId.getAsInt()),
        Commands.either(playAprilTagFoundSound(), Commands.none(), () -> targetTagId.get() > 0));
  }

  private Command rotateRelativeDegrees(double degrees) {
    double targetRadians = Math.abs(Math.toRadians(degrees));
    double direction = Math.signum(degrees) == 0.0 ? 1.0 : Math.signum(degrees);
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);

    return subsystem
        .startRun(
            () -> {
              lastHeadingRad.set(subsystem.getHeading().getRadians());
              rotatedRad.set(0.0);
              debugAuto(String.format("ROTATE START degrees=%.1f", degrees));
            },
            () -> {
              double currentHeading = subsystem.getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);
              swerveDrive.drive(
                  new Translation2d(0, 0), direction * SEARCH_ROTATION_RAD_PER_SEC, false, false);
            })
        .until(() -> rotatedRad.get() >= targetRadians)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "ROTATE END target=%.1f actual=%.1f",
                      degrees, Math.toDegrees(rotatedRad.get())));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command findSecondTagFromCurrentTag(AtomicInteger currentTagId, AtomicInteger secondTagId) {
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
        Commands.runOnce(() -> debugAuto(String.format("CENTER TAG START tagId=%d", tagIdRef.get()))),
        Commands.either(
            alignToTargetWithRotationLimit(
                () -> getVisibleAprilTagById(tagIdRef.get()).map(TargetObservation::target),
                TAG_CENTER_TOLERANCE_DEG,
                TAG_SEARCH_MAX_RADIANS),
            Commands.none(),
            () -> tagIdRef.get() > 0),
        Commands.runOnce(() -> debugAuto(String.format("CENTER TAG END tagId=%d", tagIdRef.get()))));
  }
}
