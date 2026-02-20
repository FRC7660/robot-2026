package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /** April Tag Field Layout of the year. */
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}. */
  private final double maximumAmbiguity = 0.25;

  /** Photon Vision Simulation */
  public VisionSystemSim visionSim;

  /** Count of times that the odom thinks we're more than 10meters away from the april tag. */
  private double longDistangePoseEstimationCount = 0;

  /** Current pose from the pose estimator using wheel odometry. */
  private Supplier<Pose2d> currentPose;

  /** Field from {@link swervelib.SwerveDrive#field} */
  private Field2d field2d;

  // Vision fusion tuning/gating for autonomous odometry correction.
  private static final double STALE_TIMESTAMP_EPSILON_SEC = 1e-4;
  private static final int MIN_TAGS_FOR_DIRECT_ACCEPT = 2;
  private static final double MAX_SINGLE_TAG_TRANSLATION_ERROR_METERS = 0.8;
  private static final double MAX_TRANSLATION_OUTLIER_METERS = 1.5;
  private static final double MAX_STD_XY_METERS = 5.0;
  private static final double MAX_STD_THETA_RAD = 10.0;
  private static final double FUSION_STATUS_LOG_PERIOD_SEC = 1.0;

  private final EnumMap<Cameras, Double> lastFusedTimestampSec = new EnumMap<>(Cameras.class);
  private boolean wasAutoEnabled = false;
  private double lastFusionStatusLogSec = Double.NEGATIVE_INFINITY;
  private int acceptedUpdates = 0;
  private int rejectedNoEstimate = 0;
  private int rejectedStale = 0;
  private int rejectedLowTagFar = 0;
  private int rejectedHighStd = 0;
  private int rejectedOutlier = 0;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    for (Cameras camera : Cameras.values()) {
      lastFusedTimestampSec.put(camera, Double.NEGATIVE_INFINITY);
    }

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

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
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation
        && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    boolean autoEnabled = DriverStation.isAutonomousEnabled();
    if (!autoEnabled) {
      if (wasAutoEnabled) {
        System.out.printf(
            "[YAGSLFusion][AUTO] summary accepted=%d rejected={noEstimate=%d stale=%d lowTagFar=%d highStd=%d outlier=%d}%n",
            acceptedUpdates,
            rejectedNoEstimate,
            rejectedStale,
            rejectedLowTagFar,
            rejectedHighStd,
            rejectedOutlier);
      }
      wasAutoEnabled = false;
      return;
    }

    if (!wasAutoEnabled) {
      wasAutoEnabled = true;
      acceptedUpdates = 0;
      rejectedNoEstimate = 0;
      rejectedStale = 0;
      rejectedLowTagFar = 0;
      rejectedHighStd = 0;
      rejectedOutlier = 0;
      lastFusionStatusLogSec = Double.NEGATIVE_INFINITY;
      for (Cameras camera : Cameras.values()) {
        lastFusedTimestampSec.put(camera, Double.NEGATIVE_INFINITY);
      }
      System.out.println("[YAGSLFusion][AUTO] start");
    }

    record FusionCandidate(
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

    Pose2d current = swerveDrive.getPose();
    FusionCandidate best = null;

    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isEmpty()) {
        rejectedNoEstimate++;
        continue;
      }
      var pose = poseEst.get();
      Pose2d pose2d = pose.estimatedPose.toPose2d();

      double lastTs = lastFusedTimestampSec.getOrDefault(camera, Double.NEGATIVE_INFINITY);
      if (pose.timestampSeconds <= lastTs + STALE_TIMESTAMP_EPSILON_SEC) {
        rejectedStale++;
        continue;
      }

      int tagCount = pose.targetsUsed.size();
      double translationError =
          current.getTranslation().getDistance(pose2d.getTranslation());
      if (tagCount < MIN_TAGS_FOR_DIRECT_ACCEPT
          && translationError > MAX_SINGLE_TAG_TRANSLATION_ERROR_METERS) {
        rejectedLowTagFar++;
        continue;
      }

      boolean hasStd = camera.curStdDevs != null;
      double stdX = hasStd ? camera.curStdDevs.get(0, 0) : 1.0;
      double stdY = hasStd ? camera.curStdDevs.get(1, 0) : 1.0;
      double stdTheta = hasStd ? camera.curStdDevs.get(2, 0) : 1.0;
      if (hasStd && (stdX > MAX_STD_XY_METERS || stdY > MAX_STD_XY_METERS || stdTheta > MAX_STD_THETA_RAD)) {
        rejectedHighStd++;
        continue;
      }

      if (translationError > MAX_TRANSLATION_OUTLIER_METERS) {
        rejectedOutlier++;
        continue;
      }

      double score = (stdX + stdY) + (0.5 * stdTheta) + (0.25 * translationError);
      FusionCandidate candidate =
          new FusionCandidate(
              camera,
              pose2d,
              pose.timestampSeconds,
              stdX,
              stdY,
              stdTheta,
              tagCount,
              translationError,
              hasStd,
              camera.curStdDevs,
              score);
      if (best == null || candidate.score() < best.score()) {
        best = candidate;
      }
    }

    if (best != null) {
      if (best.hasStd()) {
        swerveDrive.addVisionMeasurement(best.pose(), best.timestampSec(), best.stdDevs());
      } else {
        swerveDrive.addVisionMeasurement(best.pose(), best.timestampSec());
      }
      lastFusedTimestampSec.put(best.camera(), best.timestampSec());
      acceptedUpdates++;
      System.out.printf(
          "[YAGSLPoseUpdate][AUTO] camera=%s ts=%.3f tags=%d err=%.3f pose=(%.3f, %.3f, %.1fdeg) std=(%.3f, %.3f, %.3f)%n",
          best.camera().name(),
          best.timestampSec(),
          best.tagCount(),
          best.translationError(),
          best.pose().getX(),
          best.pose().getY(),
          best.pose().getRotation().getDegrees(),
          best.stdX(),
          best.stdY(),
          best.stdTheta());
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastFusionStatusLogSec >= FUSION_STATUS_LOG_PERIOD_SEC) {
      lastFusionStatusLogSec = now;
      System.out.printf(
          "[YAGSLFusion][AUTO] accepted=%d rejected={noEstimate=%d stale=%d lowTagFar=%d highStd=%d outlier=%d}%n",
          acceptedUpdates,
          rejectedNoEstimate,
          rejectedStale,
          rejectedLowTagFar,
          rejectedHighStd,
          rejectedOutlier);
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   *
   * <ul>
   *   <li>No Pose Estimates could be generated
   *   <li>The generated pose estimate was considered not accurate
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out
   * distances more than 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  @Deprecated(since = "2024", forRemoval = true)
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
    if (pose.isPresent()) {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed) {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
          bestTargetAmbiguity = ambiguity;
        }
      }
      // ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity) {
        return Optional.empty();
      }

      // est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d())
          > 1) {
        longDistangePoseEstimationCount++;

        // if it calculates that were 10 meter away for more than 10 times in a row its
        // probably
        // right
        if (longDistangePoseEstimationCount < 10) {
          return Optional.empty();
        }
      } else {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
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
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on
   * localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      // try
      // {
      // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      // e.printStackTrace();
      // }
    }
  }

  /** Update the {@link Field2d} to include tracked targets/ */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
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

  /** Camera Enum to select each camera */
  enum Cameras {
    /** Front Camera (camera1) */
    FRONT_CAMERA(
        "camera1",
        new Rotation3d(0, Math.toRadians(0), 0),
        new Translation3d(
            Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(12)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)),
    /** Right Camera */
    // RIGHT_CAM(
    // "right",
    // new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
    // new Translation3d(
    // Units.inchesToMeters(12.056),
    // Units.inchesToMeters(-10.981),
    // Units.inchesToMeters(8.44)),
    // VecBuilder.fill(4, 4, 8),
    // VecBuilder.fill(0.5, 0.5, 1)),
    /** Center Camera */
    CENTER_CAM(
        "camB",
        new Rotation3d(0, Units.degreesToRadians(18), 0),
        new Translation3d(
            Units.inchesToMeters(-4.628),
            Units.inchesToMeters(-10.687),
            Units.inchesToMeters(16.129)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)),
    /** Back Camera (camera0) */
    BACK_CAMERA(
        "camera0",
        new Rotation3d(0, Math.toRadians(5), Math.toRadians(180)),
        new Translation3d(
            Units.inchesToMeters(-12), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));

    // Backward-compatible aliases used throughout existing code.
    public static final Cameras CAMERA0 = BACK_CAMERA;
    public static final Cameras CAMERA1 = FRONT_CAMERA;

    /** Latency alert to use when high latency is detected. */
    public final Alert latencyAlert;

    /** Camera instance for comms. */
    public final PhotonCamera camera;

    /** Pose estimator for camera. */
    public final PhotonPoseEstimator poseEstimator;

    /** Standard Deviation for single tag readings for pose estimation. */
    private final Matrix<N3, N1> singleTagStdDevs;

    /** Standard deviation for multi-tag readings for pose estimation. */
    private final Matrix<N3, N1> multiTagStdDevs;

    /** Transform of the camera rotation and translation relative to the center of the robot */
    private final Transform3d robotToCamTransform;

    /** Current standard deviations used. */
    public Matrix<N3, N1> curStdDevs;

    /** Estimated robot pose. */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /** Simulated camera instance which only exists during simulations. */
    public PhotonCameraSim cameraSim;

    /** Results list to be updated periodically and cached to avoid unnecessary queries. */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /** Last read from the camera timestamp to prevent lag due to slow data fetches. */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /** Disable high-rate object print spam by default to protect robot loop timing. */
    private static final boolean ENABLE_OBJECT_DETECTION_DEBUG_PRINTS = false;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment
     * and determine estimation noise on an actual robot.
     *
     * @param name Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the
     *     camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the
     *     camera.
     */
    Cameras(
        String name,
        Rotation3d robotToCamRotation,
        Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert =
          new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator =
          new PhotonPoseEstimator(
              Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This
     * may not be the most recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the
     *     most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations,
     * and flushes the cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by
     * timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp =
          resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();

      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      resultsList =
          Robot.isReal()
              ? camera.getAllUnreadResults()
              : cameraSim.getCamera().getAllUnreadResults();
      resultsList.sort(
          (PhotonPipelineResult a, PhotonPipelineResult b) -> {
            return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
          });
      if (ENABLE_OBJECT_DETECTION_DEBUG_PRINTS && !resultsList.isEmpty()) {
        PhotonPipelineResult latestResult = resultsList.get(0);
        if (latestResult.hasTargets()) {
          for (PhotonTrackedTarget t : latestResult.getTargets()) {
            try {
              int classId = t.getDetectedObjectClassID();
              double conf = t.getDetectedObjectConfidence();
              double yaw = t.getYaw();
              double pitch = t.getPitch();
              System.out.printf(
                  "%s detected: classId=%d conf=%.3f yaw=%.3f pitch=%.3f%n",
                  camera.getName(), classId, conf, yaw, pitch);
            } catch (NoSuchMethodError | UnsupportedOperationException e) {
              // Ignore unsupported object-detection fields on non-object pipelines.
            }
          }
        }
      }
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved
     * with {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic
     * standard deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}
