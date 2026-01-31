package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Small helper class that reads object-detection results from a PhotonVision camera and prints
 * a compact summary to stdout once per second. Designed to be called from the robot main loop.
 */
public class PhotonObjectPrinter {
  private final PhotonCamera camera;
  private double lastPrintTime = 0.0;
  private final double periodSeconds = 1.0;

  /**
   * Create a printer for the default camera name "camera0".
   */
  public PhotonObjectPrinter() {
    this("camera0");
  }

  /**
   * Create a printer for a specific PhotonVision camera name.
   *
   * @param cameraName the photonvision camera name (e.g. "camera0")
   */
  public PhotonObjectPrinter(String cameraName) {
    camera = new PhotonCamera(cameraName);
    // initialize the timer to force the first print immediately on first call
    lastPrintTime = Timer.getFPGATimestamp() - periodSeconds;
  }

  /**
   * Call from the robot main loop (e.g. periodic()). Prints at most once per second.
   */
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrintTime < periodSeconds) {
      return;
    }
    lastPrintTime = now;

    PhotonPipelineResult result;
    try {
      result = camera.getLatestResult();
    } catch (Throwable t) {
      // Defensive: some Photon versions or simulation setups may throw; log and return
      System.out.printf("[%s] error fetching result: %s\n", camera.getName(), t.getMessage());
      return;
    }

    if (result == null) {
      System.out.printf("[%s] no result (null)\n", camera.getName());
      return;
    }

    if (!result.hasTargets()) {
      System.out.printf("[%s] no targets\n", camera.getName());
      return;
    }

    List<PhotonTrackedTarget> targets = result.getTargets();
    int i = 0;
    for (PhotonTrackedTarget t : targets) {
      String detectedInfo = "";
      try {
        // Newer PhotonVision versions provide object-detection helpers
        int classId = t.getDetectedObjectClassID();
        double conf = t.getDetectedObjectConfidence();
        detectedInfo = String.format("class=%d conf=%.3f", classId, conf);
      } catch (NoSuchMethodError | UnsupportedOperationException e) {
        // Fallback for fiducial / tag-like targets
        try {
          int fid = t.getFiducialId();
          detectedInfo = String.format("fiducial=%d", fid);
        } catch (Throwable ex) {
          detectedInfo = "unknown";
        }
      }

      double yaw = t.getYaw();
      double pitch = t.getPitch();
      double area = t.getArea();

      System.out.printf(
          "[%s] target %d: %s yaw=%.3f pitch=%.3f area=%.6f\n",
          camera.getName(), i, detectedInfo, yaw, pitch, area);

      i++;
    }
  }

  /**
   * Force an immediate print on next periodic call.
   */
  public void forceImmediatePrint() {
    lastPrintTime = Timer.getFPGATimestamp() - periodSeconds;
  }
}
