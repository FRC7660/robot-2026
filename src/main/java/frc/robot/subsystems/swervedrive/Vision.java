package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.BufferedLogger;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

/**
 * Vision pipeline for AprilTag-based pose estimation and fusion.
 *
 * <p>The pipeline runs every robot cycle (teleop + auto) and consists of: 1. {@link
 * #getCameraData()} -- fetch raw results from all cameras 2. {@link
 * #estimateCameraPosesFromAprilTags(Map)} -- run PhotonPoseEstimator on each camera's results 3.
 * {@link #selectBestPose(List, SwerveDrive)} -- filter and score candidates 4. {@link
 * #setVisionMeasurement(SwerveDrive, SelectionResult)} -- apply the best candidate to the swerve
 * drive pose estimator
 */
public class Vision {

  public enum EstimatorMode {
    OFF,
    ADVANCED,
    BASIC
  }

  // ── Data records for pipeline stages ──────────────────────────────────────

  /** Raw data from one camera in a single cycle. */
  public record CameraSnapshot(
      Cameras camera,
      List<PhotonPipelineResult> aprilTagResults,
      PhotonPipelineResult latestResult) {}

  /** Output of pose estimation for one camera. */
  public record PoseEstimationResult(
      Cameras camera,
      Optional<EstimatedRobotPose> estimatedPose,
      Matrix<N3, N1> stdDevs,
      int processedResultCount,
      double cameraLatestTimestampSec) {}

  /** A candidate that passed rejection filters. */
  public record FusionCandidate(
      Cameras camera,
      Pose2d pose,
      double timestampSec,
      double stdX,
      double stdY,
      double stdTheta,
      int tagCount,
      double translationError,
      boolean hasStd,
      Matrix<N3, N1> stdDevs,
      double score) {}

  /** Output of the selection step. */
  public record SelectionResult(
      Optional<FusionCandidate> bestCandidate,
      List<FusionCandidate> acceptedCandidates,
      int rejectedNoEstimate,
      int rejectedStale,
      int rejectedLowTagFar,
      int rejectedHighStd,
      int rejectedOutlier,
      int rejectedAmbiguity,
      int rejectedOutOfField) {}

  /** Per-camera raw AprilTag observation summary from the latest frame. */
  public record CameraTagObservation(
      int tagId, double yawDeg, double distanceMeters, double tsSec) {}

  // ── Constants ─────────────────────────────────────────────────────────────

  /** April Tag Field Layout of the year. */
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  static final double STALE_TIMESTAMP_EPSILON_SEC = 1e-4;
  static final int MIN_TAGS_FOR_DIRECT_ACCEPT = 2;
  static final double SINGLE_TAG_MAX_TRUSTED_DISTANCE_METERS = 4.0;
  static final double MAX_SINGLE_TAG_TRANSLATION_ERROR_METERS = 1.2;
  static final double MAX_TRANSLATION_OUTLIER_METERS = 1.5;
  static final double MAX_MULTI_TAG_TRANSLATION_OUTLIER_METERS = 3.0;
  static final double MAX_STD_XY_METERS = 5.0;

  /** Maximum acceptable heading standard deviation (~115 degrees). */
  static final double MAX_STD_THETA_RAD = 2.0;

  static final double MAX_SINGLE_TAG_AMBIGUITY = 0.25;
  static final int CONSECUTIVE_OUTLIER_OVERRIDE_COUNT = 15;
  static final double MAX_VISION_MEASUREMENT_AGE_SEC = 0.30;
  static final double MAX_CAMERA_LATENCY_SEC = 0.20;
  static final double MAX_SINGLE_TAG_SPEED_MPS = 2.5;
  static final double MAX_SINGLE_TAG_ANGULAR_SPEED_RAD_PER_SEC = 4.0;
  private static final double FUSION_STATUS_LOG_PERIOD_SEC = 1.0;
  private static final double PIPELINE_STATS_LOG_PERIOD_SEC = 5.0;
  private static final double POSE_LOG_TRANSLATION_DELTA_METERS = 0.05;
  private static final double POSE_LOG_ROTATION_DELTA_DEG = 2.0;
  private static final double TAG_YAW_LOG_DELTA_DEG = 1.0;
  private static final double TAG_DISTANCE_LOG_DELTA_METERS = 0.05;
  private static final double TAG_TIMESTAMP_LOG_DELTA_SEC = 0.1;
  private static final double TELEOP_TAG_RECORD_PERIOD_SEC = 0.25;
  private static final int PIPELINE_STEP_COUNT = 5;
  private static final int STEP_FETCH = 0;
  private static final int STEP_EST = 1;
  private static final int STEP_SELECT = 2;
  private static final int STEP_APPLY = 3;
  private static final int STEP_TOTAL = 4;
  private static final String[] PIPELINE_STEP_LABELS = {"fetch", "est", "select", "apply", "total"};

  // ── Instance state ────────────────────────────────────────────────────────

  /** Current pose from the pose estimator using wheel odometry. */
  private Supplier<Pose2d> currentPose;

  /** Field from {@link swervelib.SwerveDrive#field} */
  private Field2d field2d;

  private final Supplier<EstimatorMode> estimatorModeInput;
  private final Supplier<Boolean> debugTelemetryEnabledInput;
  private final EnumMap<Cameras, Double> lastFusedTimestampSec = new EnumMap<>(Cameras.class);
  private double lastFusionStatusLogSec = Double.NEGATIVE_INFINITY;
  private double lastPipelineStatsLogSec = Double.NEGATIVE_INFINITY;
  private int acceptedUpdates = 0;
  private int rejectedNoEstimate = 0;
  private int rejectedStale = 0;
  private int rejectedLowTagFar = 0;
  private int rejectedHighStd = 0;
  private int rejectedOutlier = 0;
  private int rejectedAmbiguity = 0;
  private int rejectedOutOfField = 0;
  private int consecutiveOutlierCount = 0;
  private final double[] pipelineStepMinMs = new double[PIPELINE_STEP_COUNT];
  private final double[] pipelineStepMaxMs = new double[PIPELINE_STEP_COUNT];
  private final double[] pipelineStepSumMs = new double[PIPELINE_STEP_COUNT];
  private final long[] pipelineStepCount = new long[PIPELINE_STEP_COUNT];
  private Pose2d lastLoggedOdomPose = null;
  private Pose2d lastLoggedFusedPose = null;
  private final EnumMap<Cameras, CameraTagObservation> lastLoggedCameraTagObs =
      new EnumMap<>(Cameras.class);
  // volatile: defensive guard so any future move of process() to a background thread stays safe.
  private volatile Map<Cameras, CameraSnapshot> latestCameraData = new EnumMap<>(Cameras.class);
  private double lastTeleopTagRecordLogSec = Double.NEGATIVE_INFINITY;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field Current field, should be {@link SwerveDrive#field}
   * @param estimatorModeInput user-selected estimator mode supplier
   * @param debugTelemetryEnabledInput user-selected debug telemetry enabled supplier
   */
  public Vision(
      Supplier<Pose2d> currentPose,
      Field2d field,
      Supplier<EstimatorMode> estimatorModeInput,
      Supplier<Boolean> debugTelemetryEnabledInput) {
    this.currentPose = currentPose;
    this.field2d = field;
    this.estimatorModeInput = estimatorModeInput;
    this.debugTelemetryEnabledInput = debugTelemetryEnabledInput;

    for (Cameras camera : Cameras.values()) {
      lastFusedTimestampSec.put(camera, Double.NEGATIVE_INFINITY);
    }
    for (int i = 0; i < PIPELINE_STEP_COUNT; i++) {
      pipelineStepMinMs[i] = Double.POSITIVE_INFINITY;
      pipelineStepMaxMs[i] = Double.NEGATIVE_INFINITY;
      pipelineStepSumMs[i] = 0.0;
      pipelineStepCount[i] = 0;
    }
  }

  // ── Pipeline Step 1: getCameraData() ──────────────────────────────────────

  /**
   * Fetch raw results from all cameras.
   *
   * @return a map of camera to snapshot
   */
  public Map<Cameras, CameraSnapshot> getCameraData() {
    EnumMap<Cameras, CameraSnapshot> data = new EnumMap<>(Cameras.class);
    for (Cameras camera : Cameras.values()) {
      List<PhotonPipelineResult> results = camera.getCamera().getAllUnreadResults();

      PhotonPipelineResult latestResult =
          results.isEmpty() ? null : results.get(results.size() - 1);

      data.put(camera, new CameraSnapshot(camera, results, latestResult));
    }
    return data;
  }

  // ── Pipeline Step 2: estimateCameraPosesFromAprilTags() ────────────────────────────────

  /**
   * Run pose estimation on each camera's results.
   *
   * @param cameraData the snapshots from {@link #getCameraData()}
   * @return list of pose estimation results
   */
  public List<PoseEstimationResult> estimateCameraPosesFromAprilTags(
      Map<Cameras, CameraSnapshot> cameraData) {
    List<PoseEstimationResult> results = new ArrayList<>();
    for (var entry : cameraData.entrySet()) {
      Cameras camera = entry.getKey();
      CameraSnapshot snapshot = entry.getValue();
      List<PhotonPipelineResult> aprilTagResults = snapshot.aprilTagResults();

      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      Matrix<N3, N1> stdDevs = camera.getSingleTagStdDevs();
      int processedCount = 0;

      // Feed every unread frame to the estimator so its internal timestamp tracking stays
      // accurate. Only the final (most recent) estimate is surfaced; intermediate estimates
      // are intentionally discarded because the latest frame has the freshest geometry.
      for (PhotonPipelineResult result : aprilTagResults) {
        visionEst = camera.getPoseEstimator().update(result);
        List<PhotonTrackedTarget> targets =
            visionEst.isPresent() ? visionEst.get().targetsUsed : result.getTargets();
        stdDevs =
            computeStdDevs(
                visionEst,
                targets,
                camera.getSingleTagStdDevs(),
                camera.getMultiTagStdDevs(),
                fieldLayout);
        processedCount++;
      }

      double latestTs =
          snapshot.latestResult() != null ? snapshot.latestResult().getTimestampSeconds() : 0.0;
      results.add(new PoseEstimationResult(camera, visionEst, stdDevs, processedCount, latestTs));
    }
    return results;
  }

  // ── Pipeline Step 2b: computeStdDevs() (pure static) ─────────────────────

  /**
   * Compute standard deviations for a pose estimate.
   *
   * @param estimatedPose the estimated pose (may be empty)
   * @param targets all targets in this camera frame
   * @param singleTagStdDevs base std devs for single-tag estimates
   * @param multiTagStdDevs base std devs for multi-tag estimates
   * @param fieldTags the field layout for tag pose lookups
   * @return computed standard deviations
   */
  public static Matrix<N3, N1> computeStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevs,
      AprilTagFieldLayout fieldTags) {
    if (estimatedPose.isEmpty()) {
      return singleTagStdDevs;
    }

    var estStdDevs = singleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = fieldTags.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return singleTagStdDevs;
    }

    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = multiTagStdDevs;
    }
    if (numTags == 1 && avgDist > SINGLE_TAG_MAX_TRUSTED_DISTANCE_METERS) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist / 5));
    }
    return estStdDevs;
  }

  // ── Pipeline Step 3: selectAdvancedPose() ─────────────────────────────────

  /**
   * Filter and score pose estimation results using advanced rejection filters, selecting the best
   * candidate.
   *
   * <p>Rejection filters applied in order:
   *
   * <ol>
   *   <li>No estimate available
   *   <li>Stale timestamp (already fused)
   *   <li>Out of field bounds
   *   <li>High ambiguity on single-tag estimates
   *   <li>Single tag + far from odometry
   *   <li>Standard deviations too high
   *   <li>Translation outlier (with consecutive-outlier override and multi-tag leniency)
   * </ol>
   *
   * @param estimations results from {@link #estimateCameraPosesFromAprilTags}
   * @param currentPose the current robot pose from odometry
   * @param lastFusedTimestamps last fused timestamp per camera
   * @param consecutiveOutliers number of consecutive cycles where all candidates were rejected as
   *     outliers; used to force-accept after {@link #CONSECUTIVE_OUTLIER_OVERRIDE_COUNT}
   * @return selection result with best candidate and rejection counts
   */
  static SelectionResult selectAdvancedPose(
      List<PoseEstimationResult> estimations,
      Pose2d currentPose,
      Map<Cameras, Double> lastFusedTimestamps,
      int consecutiveOutliers,
      double nowSec,
      double robotSpeedMps,
      double robotOmegaRadPerSec) {
    int rejNoEst = 0;
    int rejStale = 0;
    int rejLowTagFar = 0;
    int rejHighStd = 0;
    int rejOutlier = 0;
    int rejAmbiguity = 0;
    int rejOutOfField = 0;
    FusionCandidate best = null;
    List<FusionCandidate> acceptedCandidates = new ArrayList<>();

    double fieldLength = fieldLayout.getFieldLength();
    double fieldWidth = fieldLayout.getFieldWidth();
    boolean forceAcceptOutlier = consecutiveOutliers >= CONSECUTIVE_OUTLIER_OVERRIDE_COUNT;

    for (PoseEstimationResult est : estimations) {
      if (est.estimatedPose().isEmpty()) {
        rejNoEst++;
        continue;
      }

      var pose = est.estimatedPose().get();
      Pose2d pose2d = pose.estimatedPose.toPose2d();

      double lastTs = lastFusedTimestamps.getOrDefault(est.camera(), Double.NEGATIVE_INFINITY);
      if (pose.timestampSeconds <= lastTs + STALE_TIMESTAMP_EPSILON_SEC) {
        rejStale++;
        continue;
      }
      if ((nowSec - pose.timestampSeconds) > MAX_VISION_MEASUREMENT_AGE_SEC) {
        rejStale++;
        continue;
      }
      double cameraLatencySec = Math.max(0.0, nowSec - est.cameraLatestTimestampSec());
      if (cameraLatencySec > MAX_CAMERA_LATENCY_SEC) {
        rejStale++;
        continue;
      }

      // Reject poses outside the field
      double x = pose2d.getX();
      double y = pose2d.getY();
      if (x < 0 || x > fieldLength || y < 0 || y > fieldWidth) {
        rejOutOfField++;
        continue;
      }

      int tagCount = pose.targetsUsed.size();

      // Reject high-ambiguity single-tag estimates
      if (tagCount < MIN_TAGS_FOR_DIRECT_ACCEPT) {
        double worstAmbiguity = 0;
        for (var target : pose.targetsUsed) {
          double ambiguity = target.getPoseAmbiguity();
          if (ambiguity != -1 && ambiguity > worstAmbiguity) {
            worstAmbiguity = ambiguity;
          }
        }
        if (worstAmbiguity > MAX_SINGLE_TAG_AMBIGUITY) {
          rejAmbiguity++;
          continue;
        }
      }

      double translationError = currentPose.getTranslation().getDistance(pose2d.getTranslation());
      if (tagCount < MIN_TAGS_FOR_DIRECT_ACCEPT
          && translationError > MAX_SINGLE_TAG_TRANSLATION_ERROR_METERS) {
        rejLowTagFar++;
        continue;
      }
      if (tagCount < MIN_TAGS_FOR_DIRECT_ACCEPT
          && (robotSpeedMps > MAX_SINGLE_TAG_SPEED_MPS
              || Math.abs(robotOmegaRadPerSec) > MAX_SINGLE_TAG_ANGULAR_SPEED_RAD_PER_SEC)) {
        rejHighStd++;
        continue;
      }

      Matrix<N3, N1> stdDevs = est.stdDevs();
      if (stdDevs == null) {
        rejNoEst++;
        continue;
      }
      double stdX = stdDevs.get(0, 0);
      double stdY = stdDevs.get(1, 0);
      double stdTheta = stdDevs.get(2, 0);
      if (stdX > MAX_STD_XY_METERS || stdY > MAX_STD_XY_METERS || stdTheta > MAX_STD_THETA_RAD) {
        rejHighStd++;
        continue;
      }

      // Outlier check: multi-tag gets a more generous threshold, and consecutive
      // outlier override allows recovery when odometry has drifted significantly
      double outlierThreshold =
          tagCount >= MIN_TAGS_FOR_DIRECT_ACCEPT
              ? MAX_MULTI_TAG_TRANSLATION_OUTLIER_METERS
              : MAX_TRANSLATION_OUTLIER_METERS;
      if (translationError > outlierThreshold && !forceAcceptOutlier) {
        rejOutlier++;
        continue;
      }

      double score = (stdX + stdY) + (0.5 * stdTheta) + (0.25 * translationError);
      FusionCandidate candidate =
          new FusionCandidate(
              est.camera(),
              pose2d,
              pose.timestampSeconds,
              stdX,
              stdY,
              stdTheta,
              tagCount,
              translationError,
              true,
              stdDevs,
              score);
      acceptedCandidates.add(candidate);
      if (best == null || candidate.score() < best.score()) {
        best = candidate;
      }
    }

    acceptedCandidates.sort(
        Comparator.comparingDouble(FusionCandidate::timestampSec)
            .thenComparingDouble(FusionCandidate::score));

    return new SelectionResult(
        Optional.ofNullable(best),
        acceptedCandidates,
        rejNoEst,
        rejStale,
        rejLowTagFar,
        rejHighStd,
        rejOutlier,
        rejAmbiguity,
        rejOutOfField);
  }

  static SelectionResult selectBasicPose(
      List<PoseEstimationResult> estimations,
      Pose2d currentPose,
      Map<Cameras, Double> lastFusedTimestamps,
      double nowSec) {
    int rejNoEst = 0;
    int rejStale = 0;
    FusionCandidate best = null;
    List<FusionCandidate> acceptedCandidates = new ArrayList<>();

    for (PoseEstimationResult est : estimations) {
      if (est.estimatedPose().isEmpty()) {
        rejNoEst++;
        continue;
      }

      var pose = est.estimatedPose().get();
      double lastTs = lastFusedTimestamps.getOrDefault(est.camera(), Double.NEGATIVE_INFINITY);
      if (pose.timestampSeconds <= lastTs + STALE_TIMESTAMP_EPSILON_SEC) {
        rejStale++;
        continue;
      }
      if ((nowSec - pose.timestampSeconds) > MAX_VISION_MEASUREMENT_AGE_SEC) {
        rejStale++;
        continue;
      }
      double cameraLatencySec = Math.max(0.0, nowSec - est.cameraLatestTimestampSec());
      if (cameraLatencySec > MAX_CAMERA_LATENCY_SEC) {
        rejStale++;
        continue;
      }

      Pose2d pose2d = pose.estimatedPose.toPose2d();
      int tagCount = pose.targetsUsed.size();
      double translationError = currentPose.getTranslation().getDistance(pose2d.getTranslation());
      Matrix<N3, N1> stdDevs = est.stdDevs();
      if (stdDevs == null) {
        rejNoEst++;
        continue;
      }
      double stdX = stdDevs.get(0, 0);
      double stdY = stdDevs.get(1, 0);
      double stdTheta = stdDevs.get(2, 0);
      double score = translationError;

      FusionCandidate candidate =
          new FusionCandidate(
              est.camera(),
              pose2d,
              pose.timestampSeconds,
              stdX,
              stdY,
              stdTheta,
              tagCount,
              translationError,
              true,
              stdDevs,
              score);
      acceptedCandidates.add(candidate);
      if (best == null || candidate.score() < best.score()) {
        best = candidate;
      }
    }

    acceptedCandidates.sort(
        Comparator.comparingDouble(FusionCandidate::timestampSec)
            .thenComparingDouble(FusionCandidate::score));
    return new SelectionResult(
        Optional.ofNullable(best), acceptedCandidates, rejNoEst, rejStale, 0, 0, 0, 0, 0);
  }

  // ── Pipeline: selectBestPose (instance) ──────────────────────────────────

  /**
   * Select the best pose candidate using the current estimator mode.
   *
   * @param estimations results from {@link #estimateCameraPosesFromAprilTags}
   * @param swerveDrive the swerve drive for current pose
   * @return selection result with best candidate and rejection counts
   */
  public SelectionResult selectBestPose(
      List<PoseEstimationResult> estimations, SwerveDrive swerveDrive) {
    EstimatorMode selectedMode = estimatorModeInput == null ? null : estimatorModeInput.get();
    if (selectedMode == null) {
      selectedMode = EstimatorMode.ADVANCED;
    }
    if (selectedMode == EstimatorMode.OFF) {
      consecutiveOutlierCount = 0;
      return new SelectionResult(Optional.empty(), List.of(), 0, 0, 0, 0, 0, 0, 0);
    }
    double nowSec = Timer.getFPGATimestamp();
    var robotVelocity = swerveDrive.getRobotVelocity();
    double robotSpeedMps =
        Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    double robotOmegaRadPerSec = robotVelocity.omegaRadiansPerSecond;
    if (selectedMode == EstimatorMode.BASIC) {
      consecutiveOutlierCount = 0;
      return selectBasicPose(estimations, swerveDrive.getPose(), lastFusedTimestampSec, nowSec);
    }
    return selectAdvancedPose(
        estimations,
        swerveDrive.getPose(),
        lastFusedTimestampSec,
        consecutiveOutlierCount,
        nowSec,
        robotSpeedMps,
        robotOmegaRadPerSec);
  }

  // ── Pipeline: setVisionMeasurement ─────────────────────────────────────

  /**
   * Apply the best vision candidate to the swerve drive pose estimator.
   *
   * @param swerveDrive the swerve drive to update
   * @param selection the selection result from {@link #selectBestPose}
   */
  public void setVisionMeasurement(SwerveDrive swerveDrive, SelectionResult selection) {
    Optional<FusionCandidate> fusedCandidate = selection.bestCandidate();
    List<FusionCandidate> acceptedCandidates = selection.acceptedCandidates();

    for (FusionCandidate accepted : acceptedCandidates) {
      swerveDrive.addVisionMeasurement(
          accepted.pose(), accepted.timestampSec(), accepted.stdDevs());
      lastFusedTimestampSec.put(accepted.camera(), accepted.timestampSec());
      acceptedUpdates++;
      BufferedLogger.getInstance()
          .printf(
              "[VisionPipeline] ACCEPT camera=%s ts=%.3f tags=%d err=%.3f pose=(%.3f, %.3f, %.1fdeg) std=(%.3f, %.3f, %.3f)",
              accepted.camera().name(),
              accepted.timestampSec(),
              accepted.tagCount(),
              accepted.translationError(),
              accepted.pose().getX(),
              accepted.pose().getY(),
              accepted.pose().getRotation().getDegrees(),
              accepted.stdX(),
              accepted.stdY(),
              accepted.stdTheta());
    }

    // Track consecutive outlier rejections for override logic
    if (fusedCandidate.isPresent()) {
      consecutiveOutlierCount = 0;
    } else if (selection.rejectedOutlier() > 0) {
      consecutiveOutlierCount++;
    }

    // Accumulate rejection counters
    rejectedNoEstimate += selection.rejectedNoEstimate();
    rejectedStale += selection.rejectedStale();
    rejectedLowTagFar += selection.rejectedLowTagFar();
    rejectedHighStd += selection.rejectedHighStd();
    rejectedOutlier += selection.rejectedOutlier();
    rejectedAmbiguity += selection.rejectedAmbiguity();
    rejectedOutOfField += selection.rejectedOutOfField();
  }

  // ── Pipeline Orchestrator ─────────────────────────────────────────────────

  /**
   * Run the full vision pipeline. Runs every cycle regardless of mode (teleop + auto).
   *
   * @param swerveDrive {@link SwerveDrive} instance
   */
  public void process(SwerveDrive swerveDrive) {
    long t0 = System.nanoTime();

    // Fetch raw camera frames
    Map<Cameras, CameraSnapshot> cameraData = getCameraData();
    latestCameraData = cameraData;
    long t1 = System.nanoTime();

    // Run pose estimation on each camera
    List<PoseEstimationResult> estimations = estimateCameraPosesFromAprilTags(cameraData);
    Map<Cameras, Optional<EstimatedRobotPose>> rawLatestPoseByCamera = new EnumMap<>(Cameras.class);
    for (PoseEstimationResult est : estimations) {
      rawLatestPoseByCamera.put(est.camera(), est.estimatedPose());
    }
    long t2 = System.nanoTime();

    // Select the best pose candidate
    SelectionResult selection = selectBestPose(estimations, swerveDrive);
    long t3 = System.nanoTime();

    // Apply the vision measurement to the swerve drive
    setVisionMeasurement(swerveDrive, selection);
    long t4 = System.nanoTime();

    boolean debugTelemetryEnabled =
        debugTelemetryEnabledInput != null && Boolean.TRUE.equals(debugTelemetryEnabledInput.get());
    if (debugTelemetryEnabled) {
      publishDashboardTelemetry(
          cameraData,
          rawLatestPoseByCamera,
          estimations,
          swerveDrive.getPose(),
          selection.bestCandidate());
    }
    logAprilTagTelemetryOnChange(cameraData, swerveDrive.getPose(), selection.bestCandidate());
    logTeleopAprilTagRecord(cameraData, estimations);
    logPipelineCycle(t0, t1, t2, t3, t4);
  }

  private Optional<CameraTagObservation> getClosestTagObservation(CameraSnapshot snapshot) {
    if (snapshot == null
        || snapshot.latestResult() == null
        || !snapshot.latestResult().hasTargets()) {
      return Optional.empty();
    }

    PhotonTrackedTarget closest = null;
    double minAbsYaw = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
      if (target.getFiducialId() <= 0) {
        continue;
      }
      double absYaw = Math.abs(target.getYaw());
      if (absYaw < minAbsYaw) {
        minAbsYaw = absYaw;
        closest = target;
      }
    }
    if (closest == null) {
      return Optional.empty();
    }
    return Optional.of(
        new CameraTagObservation(
            closest.getFiducialId(),
            closest.getYaw(),
            closest.getBestCameraToTarget().getTranslation().getNorm(),
            snapshot.latestResult().getTimestampSeconds()));
  }

  private boolean poseChanged(Pose2d previous, Pose2d current) {
    if (previous == null || current == null) {
      return true;
    }
    double translationDelta = previous.getTranslation().getDistance(current.getTranslation());
    double rotationDeltaDeg =
        Math.abs(current.getRotation().minus(previous.getRotation()).getDegrees());
    return translationDelta >= POSE_LOG_TRANSLATION_DELTA_METERS
        || rotationDeltaDeg >= POSE_LOG_ROTATION_DELTA_DEG;
  }

  private boolean cameraTagObservationChanged(
      CameraTagObservation previous, CameraTagObservation current) {
    if (previous == null && current == null) {
      return false;
    }
    if (previous == null || current == null) {
      return true;
    }
    if (previous.tagId() != current.tagId()) {
      return true;
    }
    return Math.abs(previous.yawDeg() - current.yawDeg()) >= TAG_YAW_LOG_DELTA_DEG
        || Math.abs(previous.distanceMeters() - current.distanceMeters())
            >= TAG_DISTANCE_LOG_DELTA_METERS
        || Math.abs(previous.tsSec() - current.tsSec()) >= TAG_TIMESTAMP_LOG_DELTA_SEC;
  }

  private void publishPoseToDashboard(String prefix, Pose2d pose, boolean valid) {
    SmartDashboard.putBoolean(prefix + "/Valid", valid);
    if (valid && pose != null) {
      SmartDashboard.putNumber(prefix + "/X", pose.getX());
      SmartDashboard.putNumber(prefix + "/Y", pose.getY());
      SmartDashboard.putNumber(prefix + "/HeadingDeg", pose.getRotation().getDegrees());
    }
  }

  private void publishPose3dToDashboard(String prefix, Pose3d pose, boolean valid) {
    SmartDashboard.putBoolean(prefix + "/Valid", valid);
    if (valid && pose != null) {
      SmartDashboard.putNumber(prefix + "/X", pose.getX());
      SmartDashboard.putNumber(prefix + "/Y", pose.getY());
      SmartDashboard.putNumber(prefix + "/Z", pose.getZ());
      SmartDashboard.putNumber(
          prefix + "/HeadingDeg", pose.getRotation().toRotation2d().getDegrees());
    }
  }

  private void publishDashboardTelemetry(
      Map<Cameras, CameraSnapshot> cameraData,
      Map<Cameras, Optional<EstimatedRobotPose>> rawLatestPoseByCamera,
      List<PoseEstimationResult> estimations,
      Pose2d odomPose,
      Optional<FusionCandidate> fusedCandidate) {
    publishPoseToDashboard("Pose/Odom", odomPose, true);
    Pose2d fusedPose = fusedCandidate.map(FusionCandidate::pose).orElse(null);
    publishPoseToDashboard("Pose/Fused", fusedPose, fusedPose != null);

    StringBuilder tagSummary = new StringBuilder();
    for (Cameras camera : Cameras.values()) {
      Optional<CameraTagObservation> observation = getClosestTagObservation(cameraData.get(camera));
      String cameraKey = "Vision/" + camera.name();
      SmartDashboard.putBoolean(cameraKey + "/TagVisible", observation.isPresent());
      if (observation.isPresent()) {
        CameraTagObservation obs = observation.get();
        SmartDashboard.putNumber(cameraKey + "/TagId", obs.tagId());
        SmartDashboard.putNumber(cameraKey + "/YawDeg", obs.yawDeg());
        SmartDashboard.putNumber(cameraKey + "/DistanceM", obs.distanceMeters());
        SmartDashboard.putNumber(cameraKey + "/TimestampSec", obs.tsSec());
        if (tagSummary.length() > 0) {
          tagSummary.append(" | ");
        }
        tagSummary.append(
            String.format(
                "%s:id=%d yaw=%.1f dist=%.2f",
                camera.name(), obs.tagId(), obs.yawDeg(), obs.distanceMeters()));
      }

      String rawPoseKey = "Vision/" + camera.name() + "/RawLatestPose";
      Optional<EstimatedRobotPose> rawPose =
          rawLatestPoseByCamera.getOrDefault(camera, Optional.empty());
      if (rawPose.isPresent()) {
        publishPose3dToDashboard(rawPoseKey, rawPose.get().estimatedPose, true);
      } else {
        publishPose3dToDashboard(rawPoseKey, null, false);
      }
    }

    for (PoseEstimationResult est : estimations) {
      String cameraPoseKey = "Vision/" + est.camera().name() + "/DerivedPose";
      Optional<EstimatedRobotPose> estimate = est.estimatedPose();
      if (estimate.isPresent()) {
        publishPoseToDashboard(cameraPoseKey, estimate.get().estimatedPose.toPose2d(), true);
      } else {
        publishPoseToDashboard(cameraPoseKey, null, false);
      }
    }

    SmartDashboard.putString(
        "Pose/Summary",
        String.format(
            "odom=(%.2f, %.2f, %.1fdeg) fused=%s tags=[%s]",
            odomPose.getX(),
            odomPose.getY(),
            odomPose.getRotation().getDegrees(),
            fusedPose == null
                ? "none"
                : String.format(
                    "(%.2f, %.2f, %.1fdeg)",
                    fusedPose.getX(), fusedPose.getY(), fusedPose.getRotation().getDegrees()),
            tagSummary.length() == 0 ? "none" : tagSummary.toString()));
  }

  private void logAprilTagTelemetryOnChange(
      Map<Cameras, CameraSnapshot> cameraData,
      Pose2d odomPose,
      Optional<FusionCandidate> fusedCandidate) {
    EnumMap<Cameras, CameraTagObservation> currentObs = new EnumMap<>(Cameras.class);
    boolean anyTagVisible = false;
    boolean cameraObsChanged = false;
    StringBuilder tagSummary = new StringBuilder();

    for (Cameras camera : Cameras.values()) {
      Optional<CameraTagObservation> obs = getClosestTagObservation(cameraData.get(camera));
      CameraTagObservation current = obs.orElse(null);
      currentObs.put(camera, current);
      if (current != null) {
        anyTagVisible = true;
        if (tagSummary.length() > 0) {
          tagSummary.append(" | ");
        }
        tagSummary.append(
            String.format(
                "%s:id=%d yaw=%.2f dist=%.2f ts=%.3f",
                camera.name(),
                current.tagId(),
                current.yawDeg(),
                current.distanceMeters(),
                current.tsSec()));
      }
      if (cameraTagObservationChanged(lastLoggedCameraTagObs.get(camera), current)) {
        cameraObsChanged = true;
      }
    }

    Pose2d fusedPose = fusedCandidate.map(FusionCandidate::pose).orElse(null);
    boolean odomChanged = poseChanged(lastLoggedOdomPose, odomPose);
    boolean fusedChanged = fusedPose != null && poseChanged(lastLoggedFusedPose, fusedPose);

    if (anyTagVisible && (odomChanged || fusedChanged || cameraObsChanged)) {
      BufferedLogger.getInstance()
          .printf(
              "[AprilTagTeleop] odom=(%.3f, %.3f, %.1fdeg) fused=%s tags=[%s]",
              odomPose.getX(),
              odomPose.getY(),
              odomPose.getRotation().getDegrees(),
              fusedPose == null
                  ? "none"
                  : String.format(
                      "(%.3f, %.3f, %.1fdeg)",
                      fusedPose.getX(), fusedPose.getY(), fusedPose.getRotation().getDegrees()),
              tagSummary.toString());

      lastLoggedOdomPose = odomPose;
      if (fusedPose != null) {
        lastLoggedFusedPose = fusedPose;
      }
      for (Cameras camera : Cameras.values()) {
        lastLoggedCameraTagObs.put(camera, currentObs.get(camera));
      }
    }
  }

  private void logTeleopAprilTagRecord(
      Map<Cameras, CameraSnapshot> cameraData, List<PoseEstimationResult> estimations) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastTeleopTagRecordLogSec < TELEOP_TAG_RECORD_PERIOD_SEC) {
      return;
    }

    StringBuilder sb = new StringBuilder();
    boolean sawAnyTag = false;

    for (Cameras camera : Cameras.values()) {
      CameraSnapshot snapshot = cameraData.get(camera);
      if (snapshot == null
          || snapshot.latestResult() == null
          || !snapshot.latestResult().hasTargets()) {
        continue;
      }

      StringBuilder camTags = new StringBuilder();
      for (PhotonTrackedTarget target : snapshot.latestResult().getTargets()) {
        int tagId = target.getFiducialId();
        if (tagId <= 0) {
          continue;
        }
        sawAnyTag = true;
        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
        String tagPoseText =
            tagPose.isPresent()
                ? String.format(
                    "(x=%.2f,y=%.2f,z=%.2f)",
                    tagPose.get().getX(), tagPose.get().getY(), tagPose.get().getZ())
                : "(unknown)";
        if (camTags.length() > 0) {
          camTags.append(", ");
        }
        camTags.append(
            String.format(
                "id=%d yaw=%.1f dist=%.2f fieldPose=%s",
                tagId, target.getYaw(), dist, tagPoseText));
      }

      if (camTags.length() == 0) {
        continue;
      }

      Optional<EstimatedRobotPose> estPose = Optional.empty();
      for (PoseEstimationResult est : estimations) {
        if (est.camera() == camera) {
          estPose = est.estimatedPose();
          break;
        }
      }

      String robotPoseText =
          estPose.isPresent()
              ? String.format(
                  "(x=%.3f,y=%.3f,z=%.3f)",
                  estPose.get().estimatedPose.getX(),
                  estPose.get().estimatedPose.getY(),
                  estPose.get().estimatedPose.getZ())
              : "none";

      if (sb.length() > 0) {
        sb.append(" | ");
      }
      sb.append(
          String.format(
              "%s robotPose=%s tags=[%s]", camera.name(), robotPoseText, camTags.toString()));
    }

    if (sawAnyTag) {
      BufferedLogger.getInstance().printf("[AprilTagTeleopRecord] t=%.3f %s", now, sb.toString());
      lastTeleopTagRecordLogSec = now;
    }
  }

  /**
   * Log pipeline timing and periodic summary.
   *
   * @param t0 start of fetch (nanos)
   * @param t1 end of fetch / start of estimation (nanos)
   * @param t2 end of estimation / start of selection (nanos)
   * @param t3 end of selection / start of apply (nanos)
   * @param t4 end of apply (nanos)
   */
  private void logPipelineCycle(long t0, long t1, long t2, long t3, long t4) {
    double fetchMs = (t1 - t0) / 1e6;
    double estMs = (t2 - t1) / 1e6;
    double selectMs = (t3 - t2) / 1e6;
    double applyMs = (t4 - t3) / 1e6;
    double totalMs = (t4 - t0) / 1e6;
    recordPipelineStepTiming(STEP_FETCH, fetchMs);
    recordPipelineStepTiming(STEP_EST, estMs);
    recordPipelineStepTiming(STEP_SELECT, selectMs);
    recordPipelineStepTiming(STEP_APPLY, applyMs);
    recordPipelineStepTiming(STEP_TOTAL, totalMs);

    if (totalMs > 5.0) {
      BufferedLogger.getInstance()
          .printf(
              "[VisionPipeline] SLOW cycle timing=[fetch=%.1fms est=%.1fms select=%.1fms apply=%.1fms total=%.1fms]",
              fetchMs, estMs, selectMs, applyMs, totalMs);
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastFusionStatusLogSec >= FUSION_STATUS_LOG_PERIOD_SEC) {
      lastFusionStatusLogSec = now;
      BufferedLogger.getInstance()
          .printf(
              "[VisionPipeline] accepted=%d rejected={noEst=%d stale=%d outOfField=%d ambiguity=%d lowTagFar=%d highStd=%d outlier=%d} consecutiveOutlier=%d",
              acceptedUpdates,
              rejectedNoEstimate,
              rejectedStale,
              rejectedOutOfField,
              rejectedAmbiguity,
              rejectedLowTagFar,
              rejectedHighStd,
              rejectedOutlier,
              consecutiveOutlierCount);
    }

    if (now - lastPipelineStatsLogSec >= PIPELINE_STATS_LOG_PERIOD_SEC) {
      lastPipelineStatsLogSec = now;
      System.out.printf("[VisionPipeline] stats %s%n", formatPipelineStepStats());
    }
  }

  private void recordPipelineStepTiming(int stepIndex, double ms) {
    pipelineStepMinMs[stepIndex] = Math.min(pipelineStepMinMs[stepIndex], ms);
    pipelineStepMaxMs[stepIndex] = Math.max(pipelineStepMaxMs[stepIndex], ms);
    pipelineStepSumMs[stepIndex] += ms;
    pipelineStepCount[stepIndex]++;
  }

  private String formatPipelineStepStats() {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < PIPELINE_STEP_COUNT; i++) {
      if (i > 0) {
        sb.append(" ");
      }
      long count = pipelineStepCount[i];
      double minMs = count > 0 ? pipelineStepMinMs[i] : Double.NaN;
      double maxMs = count > 0 ? pipelineStepMaxMs[i] : Double.NaN;
      double avgMs = count > 0 ? (pipelineStepSumMs[i] / count) : Double.NaN;
      sb.append(
          String.format(
              "%s={min=%.2fms max=%.2fms avg=%.2fms count=%d}",
              PIPELINE_STEP_LABELS[i], minMs, maxMs, avgMs, count));
    }
    return sb.toString();
  }

  /**
   * Get the camera data from the most recent {@link #process} cycle.
   *
   * @return the latest camera snapshot map
   */
  public Map<Cameras, CameraSnapshot> getLatestCameraData() {
    return latestCameraData;
  }

  // ── Legacy methods (still used by other subsystems) ───────────────────────

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the
   *     robot to position itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException(
          "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d()))
        .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    CameraSnapshot snapshot = latestCameraData.get(camera);
    if (snapshot == null) {
      return null;
    }
    for (PhotonPipelineResult result : snapshot.aprilTagResults()) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return null;
  }

  /** Update the {@link Field2d} to include tracked targets. */
  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      CameraSnapshot snapshot = latestCameraData.get(c);
      if (snapshot != null
          && snapshot.latestResult() != null
          && snapshot.latestResult().hasTargets()) {
        targets.addAll(snapshot.latestResult().targets);
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }
}
