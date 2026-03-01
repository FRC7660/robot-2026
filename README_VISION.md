# Vision and FuelPalantir

## Architecture and Data Flow

This pipeline takes AprilTag data from both cameras, fuses it into robot pose updates, and exposes telemetry to both dashboard and stdout.
The same camera snapshot stream also feeds the pure `fuelPalantir(...)` decision step used by teleop and PathPlanner-triggered commands.

```text
Robot.autonomousInit()
  -> RobotContainer.prepareAutonomous()
    -> AutonomousManager.prepareAutonomousStart() [no manual odometry reset]
  -> RobotContainer.getAutonomousCommand()
    -> AutonomousManager.getAutonomousCommand()
      -> PathPlannerAuto(<selected .auto>)
         -> AutoBuilder.configure(...) in SwerveSubsystem
            pose supplier: getPose()
            pose reset hook: resetPose()
            speeds supplier: getRobotRelativeSpeeds()
            drive consumer: setChassisSpeeds/drive with feedforwards
            red alliance mirroring supplier

PathPlanner .auto command sequence
  -> stage commands (path + named command + path + ...)
  -> may include resetOdom=true (PathPlanner-owned start pose reset)
  -> NamedCommands hooks: FuelPalantir, LogoFuelPalantir, ResetPoseFromAprilTags

BACK_CAMERA + FRONT_CAMERA
  -> Vision.process(swerveDrive) each robot loop
     1) getCameraData()          fetch unread frames
     2) estimateCameraPosesFromAprilTags()    run Photon pose estimation per camera
     3) selectBestPose()         advanced/basic filtering (stale/outlier/ambiguity/etc)
     4) setVisionMeasurement()   apply accepted pose(s) with measurement timestamp(s)
  -> fused pose updates into swerve pose estimator
  -> telemetry: SmartDashboard + [VisionPipeline]/[AprilTagVision] logs
  -> latest camera snapshots reused by FuelPalantir logic
```

Autonomous starts from the selected PathPlanner auto on the dashboard. The robot boots with a default constructor pose, but the meaningful autonomous starting pose is owned by the selected `.auto` file (when `resetOdom` is enabled) and applied through AutoBuilder at auto start. After that, the autonomous flow progresses through the ordered commands in the `.auto` sequence (paths and optional named commands), while odometry is continuously corrected by timestamped AprilTag vision fusion.

Stage 1 is autonomous initialization and command selection. `Robot.autonomousInit()` enables brake mode, runs `prepareAutonomous()` (which now only prepares/logs and does not force a second pose reset path), then schedules the selected PathPlanner auto command. This keeps reset authority centralized in PathPlanner instead of split between robot code and auto JSON.

Stage 2 is initial pose establishment and first path execution. When the selected `.auto` has `resetOdom=true`, PathPlanner applies the start pose from the first path and begins closed-loop path following through AutoBuilder using drivetrain pose/speed suppliers. Alliance mirroring is handled by the AutoBuilder red-alliance supplier so the same auto definition can run on either side.

Stage 3 is mid-auto action execution (if present in the `.auto`). Named commands such as `FuelPalantir`, `LogoFuelPalantir`, or `ResetPoseFromAprilTags` can run between path segments. `FuelPalantir` uses the latest camera snapshots for target locking/approach behavior, and `ResetPoseFromAprilTags` now injects timestamped vision measurements rather than hard-resetting odometry.

Stage 4 is subsequent path segments and completion. The remaining path commands execute in sequence until the auto command ends or is interrupted. During all stages, the vision pipeline continues to process both cameras, reject poor measurements, and fuse accepted measurements into the drivetrain pose estimator to reduce drift.

## Vision Loop Stages (`Vision.process`)

Each robot loop, `Vision.process(swerveDrive)` runs the same pipeline in the same order.

Stage 1 is frame collection (`getCameraData`). For each configured camera, the code pulls unread PhotonVision result objects (`PhotonPipelineResult`) generated since the previous loop. Each result object includes a timestamp, a has-targets flag, and a list of detected target objects (`PhotonTrackedTarget`). For AprilTag targets, the target object carries key fields such as fiducial/tag ID, yaw and pitch angles, pose ambiguity, area/skew-like image metrics, and camera-to-target transforms (`bestCameraToTarget` / alternate transform). The loop stores this in a per-camera snapshot object (`CameraSnapshot`) with three pieces: camera ID, all unread results for this loop, and the most recent result (`latestResult`) for quick downstream access.

Stage 2 is per-camera pose estimation (`estimateCameraPosesFromAprilTags`). A frame here means one PhotonVision pipeline result (`PhotonPipelineResult`) from a single camera image at a specific timestamp, containing target detections (for example tag IDs, yaw/pitch, ambiguity, and camera-to-target transforms). For each camera, unread frames (new results since the previous robot loop) are fed through Photon pose estimation so each camera produces its latest estimated robot pose and an associated standard deviation estimate.

Stage 3 is candidate filtering and selection (`selectBestPose`). The pipeline evaluates each camera estimate with the selected estimator mode (advanced/basic), rejects bad candidates (for example stale timestamps, out-of-field, high ambiguity, excessive uncertainty, outliers), and keeps accepted candidates sorted by timestamp.

Stage 4 is measurement application (`setVisionMeasurement`). Accepted candidates are applied to the drivetrain pose estimator using `addVisionMeasurement(...)` with the camera measurement timestamp (and covariance), then per-camera fused timestamps and acceptance/rejection counters are updated.

Stage 5 is telemetry and diagnostics. The loop publishes dashboard telemetry (when enabled), logs AprilTag observations and fused/odom summaries, and records per-step timing plus periodic pipeline stats so latency and filter behavior can be monitored on real hardware.

## Vision Estimator Chooser

Dashboard chooser key:

- `Vision/EstimatorMode`

Possible options:

- `Advanced (Default)` -> uses the full filter/scoring pipeline (`selectAdvancedPose`).
- `Basic` -> uses a simplified filter (`selectBasicPose`) that only rejects empty/stale estimates and picks the lowest translation-error candidate.

Current selected mode is echoed to:

- `Vision/EstimatorMode/Selected`

## Dashboard Configuration Items

These are chooser/config entries you can change from the dashboard:

- `Auto Chooser`
  - Published in `src/main/java/frc/robot/autonomous/AutonomousManager.java`
  - Backed by `AutoBuilder.buildAutoChooser()`
- `Vision/EstimatorMode`
  - Published in `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java`
  - Options:
    - `Advanced (Default)`
    - `Basic`
    - `Off (No Pose Updates)`
- `Vision/DebugTelemetry`
  - Published in `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java`
  - Options:
    - `On (Default)`
    - `Off`

## Dashboard Output Variables

Vision and pose outputs (from `src/main/java/frc/robot/subsystems/swervedrive/Vision.java`):

- `Pose/Odom/Valid`
- `Pose/Odom/X`
- `Pose/Odom/Y`
- `Pose/Odom/HeadingDeg`
- `Pose/Fused/Valid`
- `Pose/Fused/X`
- `Pose/Fused/Y`
- `Pose/Fused/HeadingDeg`
- `Pose/Summary`

Per-camera raw AprilTag observation outputs:

- `Vision/BACK_CAMERA/TagVisible`
- `Vision/BACK_CAMERA/TagId`
- `Vision/BACK_CAMERA/YawDeg`
- `Vision/BACK_CAMERA/DistanceM`
- `Vision/BACK_CAMERA/TimestampSec`
- `Vision/FRONT_CAMERA/TagVisible`
- `Vision/FRONT_CAMERA/TagId`
- `Vision/FRONT_CAMERA/YawDeg`
- `Vision/FRONT_CAMERA/DistanceM`
- `Vision/FRONT_CAMERA/TimestampSec`

Per-camera derived robot pose outputs:

- `Vision/BACK_CAMERA/DerivedPose/Valid`
- `Vision/BACK_CAMERA/DerivedPose/X`
- `Vision/BACK_CAMERA/DerivedPose/Y`
- `Vision/BACK_CAMERA/DerivedPose/HeadingDeg`
- `Vision/FRONT_CAMERA/DerivedPose/Valid`
- `Vision/FRONT_CAMERA/DerivedPose/X`
- `Vision/FRONT_CAMERA/DerivedPose/Y`
- `Vision/FRONT_CAMERA/DerivedPose/HeadingDeg`

Other existing dashboard outputs in this project:

- `Turret/SetpointRad`
- `Turret/TargetX`
- `Turret/TargetY`
- `LimitedTranslation`
- `Translation`
- `At Tolerance`

## How FuelPalantir Works With PathPlanner

Command registration:

- `FuelPalantir`
- `LogoFuelPalantir`
- `ResetPoseFromAprilTags`

These are registered in `src/main/java/frc/robot/RobotContainer.java` using `NamedCommands.registerCommand(...)`.

FuelPalantir internals:

- Pure decision step (testable): `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java` static function `fuelPalantir(...)`
- Runtime command wrapper: `fuelPalantirCommand(...)`
- AprilTag odometry reset: `resetOdometryFromAprilTags()`

Mode behavior:

- `AUTONOMOUS`: runs for up to 15s, then completes.
- `TELEOP`: runs until cancelled (button release).

## Choosing the PathPlanner Auto That Uses FuelPalantir

1. Build an auto in the PathPlanner GUI that includes:
   - first path segment
   - named command `FuelPalantir` or `LogoFuelPalantir`
   - named command `ResetPoseFromAprilTags` (if desired before segment 2)
   - second path segment
2. Save it; this writes an `.auto` file under `src/main/deploy/pathplanner/autos/`.
3. Deploy robot code.
4. On dashboard, pick that auto from `Auto Chooser`.

## Where the Split Path Code Lives

- Split sequencing for PathPlanner autos is in PathPlanner auto JSON files:
  - `src/main/deploy/pathplanner/autos/*.auto`
- Current configured split example is `togo.auto` using:
  - `path_to_center`
  - `path_to_alliance`
- The Java side provides the named command hooks in `RobotContainer`; the split itself is authored in the `.auto` file.

## Where PathPlanner Files Go

- Path files: `src/main/deploy/pathplanner/paths/*.path`
- Auto files: `src/main/deploy/pathplanner/autos/*.auto`
- Generated JSON: `src/main/deploy/pathplanner/generatedJSON/*.wpilib.json`
