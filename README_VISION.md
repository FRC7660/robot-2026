# Vision and FuelPalantir

## Architecture and Data Flow

This pipeline takes AprilTag data from both cameras, fuses it into robot pose updates, and exposes telemetry to both dashboard and stdout.
The same camera snapshot stream also feeds the pure `fuelPalantir(...)` decision step used by teleop and PathPlanner-triggered commands.

```text
BACK_CAMERA  ----\
                  \
FRONT_CAMERA ------> Vision.process(swerveDrive)
                  |
                  |  1. getCameraData()           -- fetch raw frames
                  |  2. updateAprilTagError()     -- run pose estimation per camera
                  |  3. selectBestPose()          -- filter/score candidates (advanced or basic)
                  |  4. setVisionMeasurement()    -- apply best pose to swerve drive
                  |
                  +--> fused pose ---> swerveDrive.addVisionMeasurement(...)
                  +--> dashboard: Pose/*, Vision/*
                  +--> stdout: [VisionPipeline], [AprilTagTeleop]
                  +--> returns CameraSnapshot map ---> fuelPalantir(...) [pure static]
                                                     ---> fuelPalantirCommand(...) ---> swerveDrive.drive(...)

PathPlanner .auto ---> NamedCommands (FuelPalantir*, ResetPoseFromAprilTags)
                   ---> fuelPalantirCommand(...)
```

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
- `Pose Init`
  - Published in `src/main/java/frc/robot/RobotContainer.java`
  - Options:
    - `PathPlanner path start`
    - `DriverStation alliance position`
    - `Zero (origin)`

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
