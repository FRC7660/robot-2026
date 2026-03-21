package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Camera Enum to select each camera */
public enum Cameras {
  /** Left Front Camera */
  FRONT_LEFT(
      "camFL",
      new Rotation3d(0, Math.toRadians(-13.9), Math.toRadians(46.6575)),
      new Translation3d(
          Units.inchesToMeters(-0.243), Units.inchesToMeters(7.376), Units.inchesToMeters(18.222)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1)),
  /** Right Front Camera */
  FRONT_RIGHT(
      "camFR",
      new Rotation3d(0, Math.toRadians(-13.9), Math.toRadians(-46.6575)),
      new Translation3d(
          Units.inchesToMeters(-0.243), Units.inchesToMeters(-7.376), Units.inchesToMeters(18.222)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1)),
  /** Left Back Camera */
  BACK_LEFT(
      "camBL",
      new Rotation3d(0, Math.toRadians(-25), Math.toRadians(225)),
      new Translation3d(
          Units.inchesToMeters(-9.8), Units.inchesToMeters(12.31), Units.inchesToMeters(13.9)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1)),
  /** Right Back Camera */
  BACK_RIGHT(
      "camBR",
      new Rotation3d(0, Math.toRadians(-25), Math.toRadians(135)),
      new Translation3d(
          Units.inchesToMeters(-9.8), Units.inchesToMeters(-12.31), Units.inchesToMeters(13.9)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1));

  /** Name used to look up the PhotonVision camera. */
  private final String name;

  /** Standard Deviation for single tag readings for pose estimation. */
  private final Matrix<N3, N1> singleTagStdDevs;

  /** Standard deviation for multi-tag readings for pose estimation. */
  private final Matrix<N3, N1> multiTagStdDevs;

  /** Transform of the camera rotation and translation relative to the center of the robot */
  final Transform3d robotToCamTransform;

  // Lazily initialized hardware fields (require native libraries)
  private Alert latencyAlert;
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;

  /** Simulated camera instance which only exists during simulations. */
  public PhotonCameraSim cameraSim;

  /**
   * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and
   * determine estimation noise on an actual robot.
   *
   * @param name Name of the PhotonVision camera found in the PV UI.
   * @param robotToCamRotation {@link Rotation3d} of the camera.
   * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
   * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the camera.
   * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the
   *     camera.
   */
  Cameras(
      String name,
      Rotation3d robotToCamRotation,
      Translation3d robotToCamTranslation,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevsMatrix) {
    this.name = name;

    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

    this.singleTagStdDevs = singleTagStdDevs;
    this.multiTagStdDevs = multiTagStdDevsMatrix;
  }

  private synchronized void ensureInitialized() {
    if (camera == null) {
      latencyAlert =
          new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      camera = new PhotonCamera(name);
      poseEstimator =
          new PhotonPoseEstimator(
              Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  private synchronized void ensureSimInitialized() {
    if (cameraSim == null) {
      ensureInitialized();
      cameraSim =
          new PhotonCameraSim(camera, SimCameraProperties.PERFECT_90DEG(), Vision.fieldLayout);
    }
  }

  /** Get the latency alert for this camera. */
  public Alert getLatencyAlert() {
    ensureInitialized();
    return latencyAlert;
  }

  /** Get the PhotonCamera instance. */
  public PhotonCamera getCamera() {
    ensureInitialized();
    return camera;
  }

  /** Get the pose estimator for this camera. */
  public PhotonPoseEstimator getPoseEstimator() {
    ensureInitialized();
    return poseEstimator;
  }

  /** Get the single-tag standard deviations for this camera. */
  Matrix<N3, N1> getSingleTagStdDevs() {
    return singleTagStdDevs;
  }

  /** Get the multi-tag standard deviations for this camera. */
  Matrix<N3, N1> getMultiTagStdDevs() {
    return multiTagStdDevs;
  }

  /**
   * Add camera to {@link VisionSystemSim} for simulated photon vision.
   *
   * @param systemSim {@link VisionSystemSim} to use.
   */
  public void addToVisionSim(VisionSystemSim systemSim) {
    if (systemSim == null) {
      return;
    }
    ensureSimInitialized();
    if (cameraSim != null) {
      systemSim.addCamera(cameraSim, robotToCamTransform);
    }
  }
}
