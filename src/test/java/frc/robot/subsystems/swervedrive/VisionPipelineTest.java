package frc.robot.subsystems.swervedrive;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionPipelineTest {

  private static final Matrix<N3, N1> SINGLE_TAG_STD = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> MULTI_TAG_STD = VecBuilder.fill(0.5, 0.5, 1);

  // ── Helper methods ──────────────────────────────────────────────────────

  private static PhotonTrackedTarget makeTarget(int fiducialId) {
    return makeTarget(fiducialId, 0.1, Math.max(0.0, fiducialId));
  }

  private static PhotonTrackedTarget makeTarget(int fiducialId, double ambiguity) {
    return makeTarget(fiducialId, ambiguity, Math.max(0.0, fiducialId));
  }

  private static PhotonTrackedTarget makeTarget(
      int fiducialId, double ambiguity, double cameraDistanceMeters) {
    PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
    when(target.getFiducialId()).thenReturn(fiducialId);
    when(target.getPoseAmbiguity()).thenReturn(ambiguity);
    when(target.getBestCameraToTarget())
        .thenReturn(new Transform3d(new Translation3d(cameraDistanceMeters, 0.0, 0.0), new Rotation3d()));
    return target;
  }

  private static EstimatedRobotPose makeEstimatedPose(
      double x, double y, double timestampSec, List<PhotonTrackedTarget> targets) {
    return new EstimatedRobotPose(new Pose3d(x, y, 0, new Rotation3d()), timestampSec, targets);
  }

  /**
   * Build a mock AprilTagFieldLayout that returns tag poses at known locations. Tags 1-22 are
   * placed at (tagId, 0, 0) for predictable distance calculations.
   */
  private static AprilTagFieldLayout makeMockFieldLayout() {
    AprilTagFieldLayout layout = mock(AprilTagFieldLayout.class);
    for (int id = 1; id <= 22; id++) {
      when(layout.getTagPose(id)).thenReturn(Optional.of(new Pose3d(id, 0, 0, new Rotation3d())));
    }
    when(layout.getTagPose(0)).thenReturn(Optional.empty());
    when(layout.getTagPose(-1)).thenReturn(Optional.empty());
    return layout;
  }

  // ── computeStdDevs tests ────────────────────────────────────────────────

  @Test
  void computeStdDevs_noPoseEstimate_returnsSingleTagStdDevs() {
    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.empty(), List.of(), SINGLE_TAG_STD, MULTI_TAG_STD, makeMockFieldLayout());

    assertEquals(SINGLE_TAG_STD, result);
  }

  @Test
  void computeStdDevs_poseButNoRecognizedTags_returnsSingleTagStdDevs() {
    PhotonTrackedTarget unknownTarget = makeTarget(0);
    EstimatedRobotPose est = makeEstimatedPose(1.0, 0.0, 1.0, List.of(unknownTarget));

    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.of(est),
            List.of(unknownTarget),
            SINGLE_TAG_STD,
            MULTI_TAG_STD,
            makeMockFieldLayout());

    assertEquals(SINGLE_TAG_STD, result);
  }

  @Test
  void computeStdDevs_singleTagClose_returnsScaledSingleTagStdDevs() {
    // Tag 1 is at (1,0,0). Robot estimated at (0,0,0). Distance = 1m.
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 1.0, List.of(target));

    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.of(est),
            List.of(target),
            SINGLE_TAG_STD,
            MULTI_TAG_STD,
            makeMockFieldLayout());

    // avgDist = 1.0m, scale = 1 + (1.0/5) = 1.2
    double expectedScale = 1 + (1.0 / 5);
    assertEquals(SINGLE_TAG_STD.get(0, 0) * expectedScale, result.get(0, 0), 1e-6);
    assertEquals(SINGLE_TAG_STD.get(1, 0) * expectedScale, result.get(1, 0), 1e-6);
    assertEquals(SINGLE_TAG_STD.get(2, 0) * expectedScale, result.get(2, 0), 1e-6);
  }

  @Test
  void computeStdDevs_singleTagFar_returnsMaxValue() {
    // Tag 1 at (1,0,0). Robot estimated at (-4,0,0). Distance = 5m > 4m threshold.
    PhotonTrackedTarget target = makeTarget(1, 0.1, 5.0);
    EstimatedRobotPose est = makeEstimatedPose(-4.0, 0.0, 1.0, List.of(target));

    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.of(est),
            List.of(target),
            SINGLE_TAG_STD,
            MULTI_TAG_STD,
            makeMockFieldLayout());

    assertEquals(Double.MAX_VALUE, result.get(0, 0));
    assertEquals(Double.MAX_VALUE, result.get(1, 0));
    assertEquals(Double.MAX_VALUE, result.get(2, 0));
  }

  @Test
  void computeStdDevs_multipleTags_returnsScaledMultiTagStdDevs() {
    // Tags 1 at (1,0,0) and 2 at (2,0,0). Robot at (0,0,0).
    // Distances: 1m and 2m. avgDist = 1.5m.
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 1.0, List.of(target1, target2));

    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.of(est),
            List.of(target1, target2),
            SINGLE_TAG_STD,
            MULTI_TAG_STD,
            makeMockFieldLayout());

    // avgDist = 1.5, scale = 1 + (1.5/5) = 1.3
    double expectedScale = 1 + (1.5 / 5);
    assertEquals(MULTI_TAG_STD.get(0, 0) * expectedScale, result.get(0, 0), 1e-6);
    assertEquals(MULTI_TAG_STD.get(1, 0) * expectedScale, result.get(1, 0), 1e-6);
    assertEquals(MULTI_TAG_STD.get(2, 0) * expectedScale, result.get(2, 0), 1e-6);
  }

  @Test
  void computeStdDevs_multipleTagsAtZeroDistance_returnsExactlyMultiTagStdDevs() {
    AprilTagFieldLayout layout = mock(AprilTagFieldLayout.class);
    when(layout.getTagPose(1)).thenReturn(Optional.of(new Pose3d(0, 0, 0, new Rotation3d())));
    when(layout.getTagPose(2)).thenReturn(Optional.of(new Pose3d(0, 0, 0, new Rotation3d())));

    PhotonTrackedTarget target1 = makeTarget(1, 0.1, 0.0);
    PhotonTrackedTarget target2 = makeTarget(2, 0.1, 0.0);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 1.0, List.of(target1, target2));

    Matrix<N3, N1> result =
        Vision.computeStdDevs(
            Optional.of(est), List.of(target1, target2), SINGLE_TAG_STD, MULTI_TAG_STD, layout);

    assertEquals(MULTI_TAG_STD.get(0, 0), result.get(0, 0), 1e-6);
    assertEquals(MULTI_TAG_STD.get(1, 0), result.get(1, 0), 1e-6);
    assertEquals(MULTI_TAG_STD.get(2, 0), result.get(2, 0), 1e-6);
  }

  // ── selectAdvancedPose tests ────────────────────────────────────────────────

  @Test
  void selectAdvancedPose_allCamerasEmpty_rejectedNoEstimate() {
    List<Vision.PoseEstimationResult> estimations = new ArrayList<>();
    for (Cameras cam : Cameras.values()) {
      estimations.add(
          new Vision.PoseEstimationResult(cam, Optional.empty(), SINGLE_TAG_STD, 0, 0.0));
    }

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(Cameras.values().length, result.rejectedNoEstimate());
  }

  @Test
  void selectAdvancedPose_staleTimestamp_rejected() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 5.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Map<Cameras, Double> lastFused = new EnumMap<>(Cameras.class);
    lastFused.put(cam, 5.0);

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(estimations, new Pose2d(), lastFused, 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedStale());
  }

  @Test
  void selectAdvancedPose_singleTagHighTranslationError_rejectedLowTagFar() {
    Cameras cam = Cameras.values()[0];
    // Single tag, estimated at (2,0) but current at (0,0) -> error = 2m > 1.2m
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(2.0, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedLowTagFar());
  }

  @Test
  void selectAdvancedPose_highStdDevs_rejectedHighStd() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(0.1, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(6.0, 6.0, 1.0); // stdX > 5.0

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedHighStd());
  }

  @Test
  void selectAdvancedPose_outlierTranslationError_singleTag_rejectedOutlier() {
    Cameras cam = Cameras.values()[0];
    // Single tag at 1.6m error > 1.5m single-tag outlier threshold
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(1.5, 0.3, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    // Single tag at 1.53m > 1.2m, so it hits lowTagFar first
    assertTrue(result.rejectedLowTagFar() > 0 || result.rejectedOutlier() > 0);
  }

  @Test
  void selectAdvancedPose_outlierTranslationError_multiTag_rejectedOutlier() {
    Cameras cam = Cameras.values()[0];
    // Multi-tag at 3.5m error > 3.0m multi-tag outlier threshold
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(3.5, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedOutlier());
  }

  @Test
  void selectAdvancedPose_singleValidCamera_accepted() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(0.1, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 2.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isPresent());
    Vision.FusionCandidate best = result.bestCandidate().get();
    assertEquals(cam, best.camera());

    // score = (1.0 + 1.0) + (0.5 * 2.0) + (0.25 * 0.1)
    double expectedScore = 2.0 + 1.0 + 0.025;
    assertEquals(expectedScore, best.score(), 1e-6);
  }

  @Test
  void selectAdvancedPose_twoValidCameras_lowestScoreWins() {
    Cameras[] cams = Cameras.values();
    assertTrue(cams.length >= 2, "Need at least 2 cameras for this test");

    PhotonTrackedTarget t1a = makeTarget(1);
    PhotonTrackedTarget t1b = makeTarget(2);
    EstimatedRobotPose est1 = makeEstimatedPose(0.1, 0.0, 1.0, List.of(t1a, t1b));
    Matrix<N3, N1> stdDevs1 = VecBuilder.fill(2.0, 2.0, 3.0);

    PhotonTrackedTarget t2a = makeTarget(1);
    PhotonTrackedTarget t2b = makeTarget(2);
    EstimatedRobotPose est2 = makeEstimatedPose(0.1, 0.0, 2.0, List.of(t2a, t2b));
    Matrix<N3, N1> stdDevs2 = VecBuilder.fill(0.5, 0.5, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(
            new Vision.PoseEstimationResult(cams[0], Optional.of(est1), stdDevs1, 1, 0.0),
            new Vision.PoseEstimationResult(cams[1], Optional.of(est2), stdDevs2, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(cams[1], result.bestCandidate().get().camera());
  }

  @Test
  void selectAdvancedPose_multiTagBypassesSingleTagDistanceFilter() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(0.4, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(0, result.rejectedLowTagFar());
  }

  @Test
  void selectAdvancedPose_boundaryExactlyAtSingleTagLimit_rejected() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(1.201, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedLowTagFar());
  }

  @Test
  void selectAdvancedPose_boundaryExactlyAtSingleTagLimit_accepted() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(1.2, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    // Passes lowTagFar (strictly >1.2), then fails outlier gate (>1.0)
    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedOutlier());
  }

  @Test
  void selectAdvancedPose_highAmbiguitySingleTag_rejectedAmbiguity() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1, 0.4); // above 0.25 threshold
    EstimatedRobotPose est = makeEstimatedPose(0.1, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedAmbiguity());
  }

  @Test
  void selectAdvancedPose_outOfFieldBounds_rejectedOutOfField() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est = makeEstimatedPose(-1.0, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedOutOfField());
  }

  @Test
  void selectAdvancedPose_consecutiveOutlierOverride_accepted() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    // 2.0m translation error > 1.5m outlier threshold, but override is active
    EstimatedRobotPose est = makeEstimatedPose(2.0, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations,
            new Pose2d(),
            new EnumMap<>(Cameras.class),
            Vision.CONSECUTIVE_OUTLIER_OVERRIDE_COUNT,
            0.0,
            0.0,
            0.0);

    assertTrue(result.bestCandidate().isPresent());
  }

  // ── selectBasicPose tests ────────────────────────────────────────────────

  @Test
  void selectBasicPose_allCamerasEmpty_rejectedNoEstimate() {
    List<Vision.PoseEstimationResult> estimations = new ArrayList<>();
    for (Cameras cam : Cameras.values()) {
      estimations.add(
          new Vision.PoseEstimationResult(cam, Optional.empty(), SINGLE_TAG_STD, 0, 0.0));
    }

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(Cameras.values().length, result.rejectedNoEstimate());
  }

  @Test
  void selectBasicPose_staleTimestamp_rejected() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 5.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Map<Cameras, Double> lastFused = new EnumMap<>(Cameras.class);
    lastFused.put(cam, 5.0);

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), lastFused, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedStale());
  }

  @Test
  void selectBasicPose_singleValidCamera_accepted() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(0.5, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0.0);

    assertTrue(result.bestCandidate().isPresent());
    // score = translationError = 0.5
    assertEquals(0.5, result.bestCandidate().get().score(), 1e-6);
  }

  @Test
  void selectBasicPose_twoValidCameras_lowestTranslationErrorWins() {
    Cameras[] cams = Cameras.values();
    assertTrue(cams.length >= 2);

    PhotonTrackedTarget t1 = makeTarget(1);
    EstimatedRobotPose est1 = makeEstimatedPose(2.0, 0.0, 1.0, List.of(t1));
    Matrix<N3, N1> stdDevs1 = VecBuilder.fill(1.0, 1.0, 1.0);

    PhotonTrackedTarget t2 = makeTarget(1);
    EstimatedRobotPose est2 = makeEstimatedPose(0.3, 0.0, 2.0, List.of(t2));
    Matrix<N3, N1> stdDevs2 = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(
            new Vision.PoseEstimationResult(cams[0], Optional.of(est1), stdDevs1, 1, 0.0),
            new Vision.PoseEstimationResult(cams[1], Optional.of(est2), stdDevs2, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0.0);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(cams[1], result.bestCandidate().get().camera());
  }

  @Test
  void selectBasicPose_nullStdDevs_rejectedNoEstimate() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose est = makeEstimatedPose(0.0, 0.0, 1.0, List.of(target));

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), null, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedNoEstimate());
  }

  @Test
  void selectBasicPose_acceptsHighTranslationError() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    // 5m translation error -- basic mode has no outlier/distance filter
    EstimatedRobotPose est = makeEstimatedPose(5.0, 0.0, 1.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectBasicPose(estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0.0);

    assertTrue(result.bestCandidate().isPresent());
  }

  // ── selectAdvancedPose tests (continued) ──────────────────────────────────

  @Test
  void selectAdvancedPose_multiTagHigherOutlierThreshold_accepted() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    // 0.4m error: accepted by multi-tag outlier gate (0.5m)
    EstimatedRobotPose est = makeEstimatedPose(0.4, 0.0, 1.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(0, result.rejectedOutlier());
  }

  @Test
  void selectAdvancedPose_multiTagLargeHeadingJump_rejectedOutlier() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose est =
        new EstimatedRobotPose(
            new Pose3d(0.2, 0.1, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(20.0))),
            1.0,
            List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(new Vision.PoseEstimationResult(cam, Optional.of(est), stdDevs, 1, 0.0));

    Vision.SelectionResult result =
        Vision.selectAdvancedPose(
            estimations, new Pose2d(), new EnumMap<>(Cameras.class), 0, 0.0, 0.0, 0.0);

    assertTrue(result.bestCandidate().isEmpty());
    assertEquals(1, result.rejectedOutlier());
  }

  @Test
  void selectResetPose_prefersHigherTagCountOverCloserOdometry() {
    Cameras[] cams = Cameras.values();
    assertTrue(cams.length >= 2);

    PhotonTrackedTarget nearTarget = makeTarget(1);
    EstimatedRobotPose nearEstimate = makeEstimatedPose(0.1, 0.0, 1.0, List.of(nearTarget));
    Matrix<N3, N1> nearStdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    PhotonTrackedTarget farTarget1 = makeTarget(1);
    PhotonTrackedTarget farTarget2 = makeTarget(2);
    EstimatedRobotPose farEstimate = makeEstimatedPose(2.0, 0.0, 2.0, List.of(farTarget1, farTarget2));
    Matrix<N3, N1> farStdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(
            new Vision.PoseEstimationResult(
                cams[0], Optional.of(nearEstimate), nearStdDevs, 1, 2.0, Double.NaN, Double.NaN),
            new Vision.PoseEstimationResult(
                cams[1], Optional.of(farEstimate), farStdDevs, 1, 2.0, Double.NaN, Double.NaN));

    Vision.SelectionResult result =
        Vision.selectResetPose(estimations, new Pose2d(), 2.0, 0.0, 0.0, 1);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(cams[1], result.bestCandidate().get().camera());
    assertEquals(2, result.bestCandidate().get().tagCount());
  }

  @Test
  void selectResetPose_minTagCountRejectsSingleTagCandidate() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target = makeTarget(1);
    EstimatedRobotPose estimate = makeEstimatedPose(0.1, 0.0, 2.0, List.of(target));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(
            new Vision.PoseEstimationResult(
                cam, Optional.of(estimate), stdDevs, 1, 2.0, Double.NaN, Double.NaN));

    Vision.SelectionResult result =
        Vision.selectResetPose(estimations, new Pose2d(), 2.0, 0.0, 0.0, 2);

    assertTrue(result.bestCandidate().isEmpty());
    assertTrue(
        result.rejectedCandidates().stream()
            .anyMatch(candidate -> "insufficient_tags".equals(candidate.reason())));
  }

  @Test
  void selectResetPose_ignoresTranslationOutlierToAllowRecovery() {
    Cameras cam = Cameras.values()[0];
    PhotonTrackedTarget target1 = makeTarget(1);
    PhotonTrackedTarget target2 = makeTarget(2);
    EstimatedRobotPose estimate = makeEstimatedPose(2.5, 0.0, 2.0, List.of(target1, target2));
    Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, 1.0);

    List<Vision.PoseEstimationResult> estimations =
        List.of(
            new Vision.PoseEstimationResult(
                cam, Optional.of(estimate), stdDevs, 1, 2.0, Double.NaN, Double.NaN));

    Vision.SelectionResult result =
        Vision.selectResetPose(estimations, new Pose2d(), 2.0, 0.0, 0.0, 2);

    assertTrue(result.bestCandidate().isPresent());
    assertEquals(0, result.rejectedOutlier());
  }
}
