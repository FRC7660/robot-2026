// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private static final double TAG_APPROACH_DISTANCE_METERS = 1.0;
  private static final double TAG_SEARCH_MAX_RADIANS = 6.0 * Math.PI; // 1080 deg
  private static final double SEARCH_ROTATION_RAD_PER_SEC = Math.toRadians(45.0);
  private static final double TAG_CENTER_TOLERANCE_DEG = 3.0;
  private static final double TAG_CENTER_NEAR_TOLERANCE_EXTRA_DEG = 1.0;
  private static final double TAG_CENTER_NEAR_HOLD_SEC = 0.7;
  private static final double BALL_CENTER_TOLERANCE_DEG = 4.0;
  private static final double APPROACH_MAX_FORWARD_MPS = 0.35;
  private static final double APPROACH_MIN_FORWARD_MPS = 0.35;
  private static final double APPROACH_DISTANCE_TOLERANCE_METERS = 0.20;
  private static final double APPROACH_STALL_DISTANCE_DELTA_METERS = 0.02;
  private static final double APPROACH_STALL_TIME_SEC = 1.8;
  private static final double ANGULAR_TRACKING_GAIN = 2.0;
  private static final double DEBUG_LOG_PERIOD_SEC = 0.25;
  private static final double LOST_TAG_HOLD_SEC = 0.4;
  private static final double LOST_TAG_RECOVERY_ROTATION_RAD_PER_SEC = Math.toRadians(16.0);
  private static final double SECOND_TAG_SEEK_FORWARD_MPS = 0.35;
  private static final double SECOND_TAG_SEEK_TIMEOUT_SEC = 4.0;
  private static final double FUEL_SPIRAL_TIMEOUT_SEC = 6.0;
  private static final double FUEL_SPIRAL_BASE_FORWARD_MPS = 0.20;
  private static final double FUEL_SPIRAL_MAX_FORWARD_MPS = 0.35;
  private static final double FUEL_SPIRAL_FORWARD_RAMP_MPS_PER_SEC = 0.04;
  private static final double FUEL_SPIRAL_LATERAL_MPS = 0.18;
  private static final double FUEL_SPIRAL_LATERAL_FREQ_HZ = 0.45;
  private static final double FUEL_SPIRAL_SPIN_PERIOD_SEC = 1.2;
  private static final double FUEL_SPIRAL_SPIN_WINDOW_SEC = 0.35;

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  /** Enable vision odometry updates while driving. */
  private final boolean visionDriveTest = true;

  /** PhotonVision class to keep an accurate odometry. */
  private Vision vision;

  /** Printer that logs photonvision object-detection once per second. */
  private PhotonObjectPrinter photonObjectPrinter;
  private double lastFrontVisionLogTimestamp = -1.0;
  private double lastBackVisionLogTimestamp = -1.0;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // We default to Blue Alliance (0) to match the default behavior of the
    // DriverStation and SwerveInputStream.
    // The correct pose will be set by Autonomous or zeroGyroWithAlliance at
    // runtime.
    boolean blueAlliance = true;
    Pose2d startingPose =
        blueAlliance
            ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
            : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being
    // created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used
    // over the internal
    // encoder and push the offsets onto it. Throws warning if not possible
    if (visionDriveTest) {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize
      // updates better.
      swerveDrive.stopOdometryThread();
    }
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(
      SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive =
        new SwerveDrive(
            driveCfg,
            controllerCfg,
            Constants.MAX_SPEED,
            new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  /** Setup the photon vision class. */
  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    // Create the object printer for camera0 and force immediate print on next loop
    photonObjectPrinter = new PhotonObjectPrinter();
    photonObjectPrinter.forceImmediatePrint();
  }

  @Override
  public void periodic() {
    logFrontBackVisionDetections();
    if (photonObjectPrinter != null) {
      photonObjectPrinter.periodic();
    }
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  private void logFrontBackVisionDetections() {
    logVisionDetectionsForCamera(Cameras.CAMERA0, true);
    logVisionDetectionsForCamera(Cameras.CAMERA1, false);
  }

  private void logVisionDetectionsForCamera(Cameras camera, boolean isFront) {
    var latest = camera.camera.getLatestResult();
    if (!latest.hasTargets()) {
      return;
    }
    double ts = latest.getTimestampSeconds();
    if (isFront) {
      if (ts <= lastFrontVisionLogTimestamp) {
        return;
      }
      lastFrontVisionLogTimestamp = ts;
    } else {
      if (ts <= lastBackVisionLogTimestamp) {
        return;
      }
      lastBackVisionLogTimestamp = ts;
    }

    String cameraName = isFront ? "front(camera0)" : "back(camera1)";
    for (PhotonTrackedTarget target : latest.getTargets()) {
      if (target.getFiducialId() > 0) {
        System.out.printf(
            "[VisionDetect][%s] AprilTag id=%d yaw=%.2f area=%.3f%n",
            cameraName,
            target.getFiducialId(),
            target.getYaw(),
            target.getArea());
      } else {
        System.out.printf(
            "[VisionDetect][%s] Fuel yaw=%.2f area=%.3f%n",
            cameraName,
            target.getYaw(),
            target.getArea());
      }
    }
  }

  @Override
  public void simulationPeriodic() {}

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive
              // trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
              ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
          );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(Cameras camera) {

    return run(
        () -> {
          Optional<PhotonPipelineResult> resultO = camera.getBestResult();
          if (resultO.isPresent()) {
            var result = resultO.get();
            if (result.hasTargets()) {
              drive(
                  getTargetSpeeds(
                      0,
                      0,
                      Rotation2d.fromDegrees(
                          result
                              .getBestTarget()
                              .getYaw()))); // Not sure if this will work, more math may be
              // required.
            }
          }
        });
  }

  /**
   * Track a detected object (e.g. a ball) using PhotonVision object-detection. The command will run
   * for {@code durationSeconds} seconds and continuously rotate the robot toward the object's yaw
   * and drive forward based on the detected area (a proxy for distance).
   *
   * <p>This is a simple proportional controller and intended as a low-risk helper that can be bound
   * to a button: e.g. schedule the returned command when a joystick button is pressed.
   *
   * @param camera the camera to read results from (use {@link Cameras})
   * @param durationSeconds how long to run the tracking for
   * @return a {@link Command} that performs the timed tracking
   */
  public Command trackDetectedObject(Cameras camera, double durationSeconds) {
    // Tunable gains and thresholds; adjust on robot for best behaviour
    final double kYawDegToRadPerSec = -1.0; // scale yaw (deg) -> deg/s then convert to rad/s
    final double maxAngularRadPerSec = Math.toRadians(180.0); // clamp rotation speed
    final double yawToleranceDeg = 10.0; // degrees within which we consider we are aimed

    final double targetArea = 60.0; // example area value considered "close" (tune)
    final double kAreaForward = 1.0; // meters/sec per area-difference (scaled below)
    final double maxForward = 0.35; // swerveDrive.getMaximumChassisVelocity() * 0.5;

    return run(() -> {
          Optional<PhotonPipelineResult> resultO =
              Optional.ofNullable(camera.camera.getLatestResult());
          double rotationRadPerSec = 0.0;
          double forwardMps = 0.0;
          System.out.println("RUN");
          if (resultO.isPresent()) {
            PhotonPipelineResult res = resultO.get();
            if (res.hasTargets()) {
              PhotonTrackedTarget t = res.getBestTarget();
              double yawDeg = t.getYaw();
              double area = t.getArea();

              // Rotation: proportional on yaw (deg -> deg/s), then convert to rad/s
              if (Math.abs(yawDeg) > yawToleranceDeg) {
                double rotDegPerSec = kYawDegToRadPerSec * yawDeg;
                rotationRadPerSec = Math.toRadians(rotDegPerSec);
                rotationRadPerSec =
                    Math.max(
                        -maxAngularRadPerSec, Math.min(maxAngularRadPerSec, rotationRadPerSec));
              } else {
                rotationRadPerSec = 0.0;
              }

              // Forward drive: if area smaller than target, drive forward proportional
              if (area < targetArea) {
                double areaError = targetArea - area;
                forwardMps = Math.min(maxForward, kAreaForward * areaError * maxForward);
              } else {
                forwardMps = 0.0;
              }
            }
          }

          // Drive: field-relative forward (x) while rotating to aim. Open-loop false
          System.out.println(forwardMps + ": " + rotationRadPerSec);
          swerveDrive.drive(
              new edu.wpi.first.math.geometry.Translation2d(forwardMps, 0.0),
              rotationRadPerSec,
              false,
              false);
        })
        .withTimeout(durationSeconds)
        .finallyDo(
            () ->
                swerveDrive.drive(
                    new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, true, false));
  }

  /**
   * Helper overload that accepts the camera enum name as a string. This is convenient for callers
   * outside the package that cannot reference the package-private {@code Vision.Cameras} type.
   *
   * @param cameraEnumName the enum constant name from {@code Vision.Cameras}, e.g. "CENTER_CAM" or
   *     "CAMERA1"
   * @param durationSeconds timeout for the tracking command
   * @return the tracking {@link Command} or {@link Commands#none()} if the name is invalid
   */
  public Command trackDetectedObjectByCameraName(String cameraEnumName, double durationSeconds) {
    System.out.println("  trackDetectedObjectByCameraName ");
    try {
      Cameras cam = Cameras.valueOf(cameraEnumName);
      return trackDetectedObject(cam, durationSeconds);
    } catch (IllegalArgumentException e) {
      // Invalid camera name: return a noop command
      return Commands.none();
    }
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            0.2, // 4.0
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
  }

  private record TargetObservation(Cameras camera, PhotonTrackedTarget target) {}

  private void debugAuto(String message) {
    System.out.printf("[AutoShuttle][%.2f] %s%n", Timer.getFPGATimestamp(), message);
  }

  private boolean shouldDebugLog(AtomicReference<Double> lastLogTimeSec, double periodSec) {
    double now = Timer.getFPGATimestamp();
    if (now - lastLogTimeSec.get() >= periodSec) {
      lastLogTimeSec.set(now);
      return true;
    }
    return false;
  }

  private Optional<PhotonTrackedTarget> getClosestDetectedObject() {
    var latest = Cameras.CAMERA0.camera.getLatestResult();
    if (!latest.hasTargets()) {
      return Optional.empty();
    }

    PhotonTrackedTarget closest = null;
    double largestArea = Double.NEGATIVE_INFINITY;
    for (PhotonTrackedTarget target : latest.getTargets()) {
      if (target.getFiducialId() > 0) {
        continue;
      }
      if (target.getArea() > largestArea) {
        largestArea = target.getArea();
        closest = target;
      }
    }
    return Optional.ofNullable(closest);
  }

  private Optional<PhotonTrackedTarget> getClosestDetectedObjectAnyCamera() {
    PhotonTrackedTarget closest = null;
    double largestArea = Double.NEGATIVE_INFINITY;
    Cameras[] fuelCameras = {Cameras.CAMERA0, Cameras.CAMERA1};
    for (Cameras camera : fuelCameras) {
      var latest = camera.camera.getLatestResult();
      if (!latest.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : latest.getTargets()) {
        if (target.getFiducialId() > 0) {
          continue;
        }
        if (target.getArea() > largestArea) {
          largestArea = target.getArea();
          closest = target;
        }
      }
    }
    return Optional.ofNullable(closest);
  }

  private double getTargetPlanarDistanceMeters(PhotonTrackedTarget target) {
    var translation = target.getBestCameraToTarget().getTranslation();
    // Use full 3D range. For some camera frames forward distance may not be encoded in X/Y only.
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
    // Negative sign so positive yaw commands correction back toward center.
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

    return startRun(
            () -> {
              lastHeadingRad.set(getHeading().getRadians());
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
              double currentHeading = getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);

              Optional<PhotonTrackedTarget> target = targetSupplier.get();
              if (target.isPresent()) {
                double yawDeg = target.get().getYaw();
                double absYaw = Math.abs(yawDeg);
                boolean hardCentered = absYaw <= centeredToleranceDeg;
                boolean nearCentered = absYaw <= centeredToleranceDeg + TAG_CENTER_NEAR_TOLERANCE_EXTRA_DEG;

                if (hardCentered) {
                  centered.set(true);
                  nearCenteredSinceSec.set(Double.NaN);
                } else if (nearCentered) {
                  if (Double.isNaN(nearCenteredSinceSec.get())) {
                    nearCenteredSinceSec.set(Timer.getFPGATimestamp());
                  }
                  centered.set(Timer.getFPGATimestamp() - nearCenteredSinceSec.get() >= TAG_CENTER_NEAR_HOLD_SEC);
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
                              : String.format("%.2f", Timer.getFPGATimestamp() - nearCenteredSinceSec.get()),
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

    return startRun(
            () -> {
              lastHeadingRad.set(getHeading().getRadians());
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
              double currentHeading = getHeading().getRadians();
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
                  || Math.abs(lastDistanceMeters.get() - distance) > APPROACH_STALL_DISTANCE_DELTA_METERS) {
                lastProgressTimeSec.set(Timer.getFPGATimestamp());
              }
              lastDistanceMeters.set(distance);
              double rotation = calculateRotationFromYawDeg(yawDeg);
              double distanceError = distance - distanceMeters;
              double forwardSpeed = 0.0;
              if (distanceError > 0.0) {
                forwardSpeed =
                    MathUtil.clamp(distanceError * 0.9, APPROACH_MIN_FORWARD_MPS, APPROACH_MAX_FORWARD_MPS);
              }
              swerveDrive.drive(new Translation2d(forwardSpeed, 0), rotation, false, false);
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
                        forwardSpeed,
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
                      "TAG SEARCH START exclude=%d max=1080deg", excludedTagId.getAsInt()));
            }),
        updateTagIdFromVisibleTarget(targetTagId, excludedTagId),
        Commands.either(
            Commands.none(),
            startRun(
                    () -> {
                      lastHeadingRad.set(getHeading().getRadians());
                      rotatedRad.set(0.0);
                      lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
                    },
                    () -> {
                      double currentHeading = getHeading().getRadians();
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
            () -> targetTagId.get() > 0 && targetTagId.get() != excludedTagId.getAsInt()));
  }

  private Command rotateRelativeDegrees(double degrees) {
    double targetRadians = Math.abs(Math.toRadians(degrees));
    double direction = Math.signum(degrees) == 0.0 ? 1.0 : Math.signum(degrees);
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);

    return startRun(
            () -> {
              lastHeadingRad.set(getHeading().getRadians());
              rotatedRad.set(0.0);
              debugAuto(String.format("ROTATE START degrees=%.1f", degrees));
            },
            () -> {
              double currentHeading = getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);
              swerveDrive.drive(
                  new Translation2d(0, 0),
                  direction * SEARCH_ROTATION_RAD_PER_SEC,
                  false,
                  false);
            })
        .until(() -> rotatedRad.get() >= targetRadians)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "ROTATE END target=%.1f actual=%.1f",
                      degrees,
                      Math.toDegrees(rotatedRad.get())));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command driveForwardUntilOtherTagSeen(
      AtomicInteger targetTagId, IntSupplier excludedTagId, double timeoutSec) {
    AtomicReference<Double> startTimeSec = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return startRun(
            () -> {
              targetTagId.set(-1);
              startTimeSec.set(Timer.getFPGATimestamp());
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              debugAuto(
                  String.format(
                      "SECOND TAG SEEK START forward=%.2fm/s timeout=%.1fs exclude=%d",
                      SECOND_TAG_SEEK_FORWARD_MPS,
                      timeoutSec,
                      excludedTagId.getAsInt()));
            },
            () -> {
              Optional<TargetObservation> candidate =
                  getClosestVisibleAprilTagObservation(excludedTagId.getAsInt());
              candidate.ifPresent(observation -> targetTagId.set(observation.target().getFiducialId()));
              swerveDrive.drive(new Translation2d(SECOND_TAG_SEEK_FORWARD_MPS, 0), 0, false, false);

              if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                if (candidate.isPresent()) {
                  debugAuto(
                      String.format(
                          "SECOND TAG SEEK found id=%d camera=%s dist=%.2fm",
                          candidate.get().target().getFiducialId(),
                          candidate.get().camera().name(),
                          getTargetPlanarDistanceMeters(candidate.get().target())));
                } else {
                  debugAuto(
                      String.format(
                          "SECOND TAG SEEK driving... elapsed=%.2fs",
                          Timer.getFPGATimestamp() - startTimeSec.get()));
                }
              }
            })
        .until(
            () ->
                targetTagId.get() > 0
                    || (Timer.getFPGATimestamp() - startTimeSec.get()) >= timeoutSec)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "SECOND TAG SEEK END selected=%d elapsed=%.2fs",
                      targetTagId.get(),
                      Timer.getFPGATimestamp() - startTimeSec.get()));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command findSecondTagFromCurrentTag(AtomicInteger currentTagId, AtomicInteger secondTagId) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                debugAuto(
                    String.format(
                        "SECOND TAG PLAN: center current=%d, rotate 180, drive seek",
                        currentTagId.get()))),
        centerOnKnownTag(currentTagId),
        rotateRelativeDegrees(180.0),
        driveForwardUntilOtherTagSeen(secondTagId, currentTagId::get, SECOND_TAG_SEEK_TIMEOUT_SEC));
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

  private Command spinSearchFuelFrontWith1080Limit(AtomicBoolean fuelFound) {
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotatedRad = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return startRun(
            () -> {
              fuelFound.set(false);
              lastHeadingRad.set(getHeading().getRadians());
              rotatedRad.set(0.0);
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              debugAuto(
                  String.format(
                      "FUEL SPIN SEARCH START fuelSeenAny=%s fuelSeenFront=%s max=1080deg",
                      getClosestDetectedObjectAnyCamera().isPresent(),
                      getClosestDetectedObject().isPresent()));
            },
            () -> {
              double currentHeading = getHeading().getRadians();
              double delta = MathUtil.angleModulus(currentHeading - lastHeadingRad.get());
              rotatedRad.set(rotatedRad.get() + Math.abs(delta));
              lastHeadingRad.set(currentHeading);

              Optional<PhotonTrackedTarget> fuelFront = getClosestDetectedObject();
              if (fuelFront.isPresent()) {
                double yawDeg = fuelFront.get().getYaw();
                double rotation = calculateRotationFromYawDeg(yawDeg);
                swerveDrive.drive(new Translation2d(0, 0), rotation, false, false);
                if (Math.abs(yawDeg) <= BALL_CENTER_TOLERANCE_DEG) {
                  fuelFound.set(true);
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL SPIN SEARCH front fuel yaw=%.2fdeg area=%.3f centered=%s rotated=%.1fdeg",
                          yawDeg,
                          fuelFront.get().getArea(),
                          fuelFound.get(),
                          Math.toDegrees(rotatedRad.get())));
                }
              } else {
                swerveDrive.drive(
                    new Translation2d(0, 0), SEARCH_ROTATION_RAD_PER_SEC, false, false);
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL SPIN SEARCH no front fuel rotated=%.1fdeg fuelSeenAny=%s",
                          Math.toDegrees(rotatedRad.get()),
                          getClosestDetectedObjectAnyCamera().isPresent()));
                }
              }
            })
        .until(() -> fuelFound.get() || rotatedRad.get() >= TAG_SEARCH_MAX_RADIANS)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "FUEL SPIN SEARCH END fuelFound=%s rotated=%.1fdeg",
                      fuelFound.get(), Math.toDegrees(rotatedRad.get())));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command spiralSearchFuelNearTag(AtomicInteger currentTagId, AtomicBoolean fuelFound) {
    AtomicReference<Double> startTimeSec = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogTimeSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);

    return startRun(
            () -> {
              startTimeSec.set(Timer.getFPGATimestamp());
              lastLogTimeSec.set(Double.NEGATIVE_INFINITY);
              debugAuto(
                  String.format(
                      "FUEL SPIRAL START tagId=%d timeout=%.1fs",
                      currentTagId.get(), FUEL_SPIRAL_TIMEOUT_SEC));
            },
            () -> {
              double elapsed = Timer.getFPGATimestamp() - startTimeSec.get();

              Optional<PhotonTrackedTarget> fuelFront = getClosestDetectedObject();
              if (fuelFront.isPresent()) {
                double yawDeg = fuelFront.get().getYaw();
                double rotation = calculateRotationFromYawDeg(yawDeg);
                swerveDrive.drive(new Translation2d(0, 0), rotation, false, false);
                if (Math.abs(yawDeg) <= BALL_CENTER_TOLERANCE_DEG) {
                  fuelFound.set(true);
                }
                if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                  debugAuto(
                      String.format(
                          "FUEL SPIRAL front fuel yaw=%.2fdeg area=%.3f centered=%s",
                          yawDeg, fuelFront.get().getArea(), fuelFound.get()));
                }
                return;
              }

              double forward =
                  Math.min(
                      FUEL_SPIRAL_MAX_FORWARD_MPS,
                      FUEL_SPIRAL_BASE_FORWARD_MPS
                          + elapsed * FUEL_SPIRAL_FORWARD_RAMP_MPS_PER_SEC);
              double lateral =
                  FUEL_SPIRAL_LATERAL_MPS
                      * Math.sin(2.0 * Math.PI * FUEL_SPIRAL_LATERAL_FREQ_HZ * elapsed);
              boolean spinWindow =
                  (elapsed % FUEL_SPIRAL_SPIN_PERIOD_SEC) <= FUEL_SPIRAL_SPIN_WINDOW_SEC;
              double rotation = spinWindow ? SEARCH_ROTATION_RAD_PER_SEC : 0.0;

              Optional<TargetObservation> tagObservation = getVisibleAprilTagById(currentTagId.get());
              if (tagObservation.isPresent() && !spinWindow) {
                rotation = calculateRotationFromYawDeg(tagObservation.get().target().getYaw());
              }

              swerveDrive.drive(new Translation2d(forward, lateral), rotation, false, false);
              if (shouldDebugLog(lastLogTimeSec, DEBUG_LOG_PERIOD_SEC)) {
                debugAuto(
                    String.format(
                        "FUEL SPIRAL drive fwd=%.2f lat=%.2f rot=%.2f elapsed=%.2f fuelSeenAny=%s tagSeen=%s",
                        forward,
                        lateral,
                        rotation,
                        elapsed,
                        getClosestDetectedObjectAnyCamera().isPresent(),
                        tagObservation.isPresent()));
              }
            })
        .until(
            () ->
                fuelFound.get()
                    || (Timer.getFPGATimestamp() - startTimeSec.get()) >= FUEL_SPIRAL_TIMEOUT_SEC)
        .finallyDo(
            () -> {
              debugAuto(
                  String.format(
                      "FUEL SPIRAL END fuelFound=%s elapsed=%.2fs",
                      fuelFound.get(), Timer.getFPGATimestamp() - startTimeSec.get()));
              swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            });
  }

  private Command findFuelForKnownTag(AtomicInteger activeTagId, AtomicBoolean fuelFound) {
    return Commands.sequence(
        Commands.runOnce(
            () -> debugAuto(String.format("FUEL ROUTINE START tagId=%d", activeTagId.get()))),
        centerOnKnownTag(activeTagId),
        spinSearchFuelFrontWith1080Limit(fuelFound),
        Commands.either(
            Commands.none(),
            spiralSearchFuelNearTag(activeTagId, fuelFound),
            fuelFound::get),
        Commands.runOnce(
            () ->
                debugAuto(
                    String.format(
                        "FUEL ROUTINE END tagId=%d fuelFound=%s",
                        activeTagId.get(), fuelFound.get()))));
  }

  public Command aprilTagBallShuttleAuto(int repetitionCount) {
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
        Commands.runOnce(() -> debugAuto(String.format("AUTO START cycles=%d", cycles))),
        scanForTagWith1080Limit(currentTagId, () -> -1),
        Commands.either(
            Commands.none(),
            Commands.runOnce(
                () -> {
                  keepRunning.set(false);
                  debugAuto("INITIAL TAG NOT FOUND after 1080deg; ending auto");
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

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator =
        new SwerveSetpointGenerator(
            RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint =
        new AtomicReference<>(
            new SwerveSetpoint(
                swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint =
              setpointGenerator.generateSetpoint(
                  prevSetpoint.get(),
                  robotRelativeChassisSpeed.get(),
                  newTime - previousTime.get());
          swerveDrive.drive(
              newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
          });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              SwerveMath.scaleTranslation(
                  new Translation2d(
                      translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                      translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                  0.8),
              Math.pow(angularRotationX.getAsDouble(), 3)
                  * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for
    // this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /** Add a fake vision reading for testing purposes. */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
        new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
