package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Camera Enum to select each camera */
public enum Cameras {
  /** Front Camera (camera1) */
  FRONT_CAMERA(
      "camera1",
      new Rotation3d(0, Math.toRadians(0), 0),
      new Translation3d(
          Units.inchesToMeters(14.5), Units.inchesToMeters(0), Units.inchesToMeters(6.25)),
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
  // CENTER_CAM disabled — "camB" not present on coprocessor, causes per-loop exceptions
  // /** Center Camera */
  // CENTER_CAM(
  //     "camB",
  //     new Rotation3d(0, Units.degreesToRadians(18), 0),
  //     new Translation3d(
  //         Units.inchesToMeters(-4.628),
  //         Units.inchesToMeters(-10.687),
  //         Units.inchesToMeters(16.129)),
  //     VecBuilder.fill(4, 4, 8),
  //     VecBuilder.fill(0.5, 0.5, 1)),
  /** Back Camera (camera0) */
  BACK_CAMERA(
      "camera0",
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)),
      new Translation3d(
          Units.inchesToMeters(-15.5), Units.inchesToMeters(0), Units.inchesToMeters(5.5)),
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
  final Transform3d robotToCamTransform;

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

  /** Get the single-tag standard deviations for this camera. */
  Matrix<N3, N1> getSingleTagStdDevs() {
    return singleTagStdDevs;
  }

  /** Get the multi-tag standard deviations for this camera. */
  Matrix<N3, N1> getMultiTagStdDevs() {
    return multiTagStdDevs;
  }
}
