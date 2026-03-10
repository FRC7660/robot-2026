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

/** Camera Enum to select each camera */
public enum Cameras {
  /** Front Camera */
  FRONT_CAMERA(
      "cam2", // front
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)),
      new Translation3d(
          Units.inchesToMeters(1.351478),
          Units.inchesToMeters(10.625),
          Units.inchesToMeters(17.860956)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1)),
  /** Right Camera */
  RIGHT_CAMERA(
      "cam0",
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(-90)),
      new Translation3d(
          Units.inchesToMeters(-9.5), Units.inchesToMeters(-7.75), Units.inchesToMeters(12.996)),
      VecBuilder.fill(4, 4, 8),
      VecBuilder.fill(0.5, 0.5, 1)),
  /** Left Camera */
  LEFT_CAMERA(
      "cam4",
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(90)),
      new Translation3d(
          Units.inchesToMeters(-9.5), Units.inchesToMeters(7.75), Units.inchesToMeters(12.996)),
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
}
