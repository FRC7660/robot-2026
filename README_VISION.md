# Vision and FuelPalantir

## Architecture and Data Flow

```text
CAMERA0 ----\
             \
CAMERA1 ------> Vision.process(...) ---> fused pose ---> swerveDrive.addVisionMeasurement(...)
             /
            +--> dashboard: Pose/*, Vision/*
            +--> stdout: [VisionPipeline], [AprilTagTeleop]
            +--> CameraSnapshot map ---> fuelPalantir(...) [pure static]
                                       ---> fuelPalantirCommand(...) ---> swerveDrive.drive(...)

PathPlanner .auto ---> NamedCommands (FuelPalantir*, ResetPoseFromAprilTags)
                   ---> fuelPalantirCommand(...)
```

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

- `Vision/CAMERA0/TagVisible`
- `Vision/CAMERA0/TagId`
- `Vision/CAMERA0/YawDeg`
- `Vision/CAMERA0/DistanceM`
- `Vision/CAMERA0/TimestampSec`
- `Vision/CAMERA1/TagVisible`
- `Vision/CAMERA1/TagId`
- `Vision/CAMERA1/YawDeg`
- `Vision/CAMERA1/DistanceM`
- `Vision/CAMERA1/TimestampSec`

Per-camera derived robot pose outputs:

- `Vision/CAMERA0/DerivedPose/Valid`
- `Vision/CAMERA0/DerivedPose/X`
- `Vision/CAMERA0/DerivedPose/Y`
- `Vision/CAMERA0/DerivedPose/HeadingDeg`
- `Vision/CAMERA1/DerivedPose/Valid`
- `Vision/CAMERA1/DerivedPose/X`
- `Vision/CAMERA1/DerivedPose/Y`
- `Vision/CAMERA1/DerivedPose/HeadingDeg`

Other existing dashboard outputs in this project:

- `Auto/NeutralToBallPickup/ProxyPickups`
- `Turret/SetpointRad`
- `Turret/TargetX`
- `Turret/TargetY`
- `LimitedTranslation`
- `Translation`
- `At Tolerance`

## How FuelPalantir Works With PathPlanner

Command registration:

- `FuelPalantirContinue30`
- `FuelPalantirStop20`
- `ResetPoseFromAprilTags`

These are registered in `src/main/java/frc/robot/RobotContainer.java` using `NamedCommands.registerCommand(...)`.

FuelPalantir internals:

- Pure decision step (testable): `src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java` static function `fuelPalantir(...)`
- Runtime command wrapper: `fuelPalantirCommand(...)`
- AprilTag odometry reset: `resetOdometryFromAprilTags()`

Mode behavior:

- `FuelPalantirContinue30`: run up to 30s, then continue.
- `FuelPalantirStop20`: run up to 20s, then hold robot stopped until disabled.
- Fuel target count is currently proxy-based with a TODO stub for real sensor/indexer integration.

## Choosing the PathPlanner Auto That Uses FuelPalantir

1. Build an auto in the PathPlanner GUI that includes:
   - first path segment
   - named command `FuelPalantirContinue30` or `FuelPalantirStop20`
   - named command `ResetPoseFromAprilTags` (if desired before segment 2)
   - second path segment
2. Save it; this writes an `.auto` file under `src/main/deploy/pathplanner/autos/`.
3. Deploy robot code.
4. On dashboard, pick that auto from `Auto Chooser`.

## Where the Split Path Code Lives

- Split sequencing for PathPlanner autos is in PathPlanner auto JSON files:
  - `src/main/deploy/pathplanner/autos/*.auto`
- Today, current autos are single-path examples (`auto1.auto`, `auto2.auto`, `togo.auto`).
- The Java side provides the named command hooks in `RobotContainer`; the split itself is authored in the `.auto` file.

## Where PathPlanner Files Go

- Path files: `src/main/deploy/pathplanner/paths/*.path`
- Auto files: `src/main/deploy/pathplanner/autos/*.auto`
- Generated JSON: `src/main/deploy/pathplanner/generatedJSON/*.wpilib.json`
