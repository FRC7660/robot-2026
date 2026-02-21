# Vision Pipeline Refactor — Feb 20, 2026

## What changed

The monolithic `Vision.updatePoseEstimation()` method was broken into a clean pipeline of explicit steps with pure functions, timing/logging, and unit tests. The vision pipeline now runs in both teleop and auto (previously auto-only).

## Pipeline architecture

```
process(swerveDrive)
  ├── getCameraData()          → Map<Cameras, CameraSnapshot>
  ├── poseEstimation(data)     → List<PoseEstimationResult>
  ├── selectBestPose(...)      → SelectionResult
  └── apply best candidate to swerveDrive
```

Each stage passes data via explicit records (`CameraSnapshot`, `PoseEstimationResult`, `FusionCandidate`, `SelectionResult`). Two key methods are **pure static** and directly unit-testable:

- `computeStdDevs()` — distance-based std dev heuristic extracted from `Cameras.updateEstimationStdDevs()`
- `selectBestPose()` — rejection filtering (stale, single-tag+far, high std, outlier) and scoring

## Pre-auto pose initialization

Replaced `resetPoseFromAprilTagsOrPath2Start()` (which tried AprilTag camera init then fell back to PathPlanner path2) with a dashboard `SendableChooser` offering three options:

- **PathPlanner path start** (default) — loads selected auto path's start pose, flips for alliance
- **DriverStation alliance position** — uses alliance + station number to look up a known field position
- **Zero (origin)** — resets to (0, 0, 0)

The AprilTag-based initialization code and `SwerveSubsystem.getAprilTagEstimatedPoseForInitialization()` (~110 lines) were deleted.

## Files modified

| File | Change |
|------|--------|
| `build.gradle` | Added mockito-core and mockito-junit-jupiter test deps |
| `Cameras.java` | Added `getSingleTagStdDevs()`, `getMultiTagStdDevs()` getters |
| `Vision.java` | Added 4 records, 4 pipeline methods, `process()` orchestrator, timing logs; removed auto-only gate and `wasAutoEnabled`; deprecated `updatePoseEstimation()` |
| `SwerveSubsystem.java` | `vision.updatePoseEstimation()` → `vision.process()`; deleted `getAprilTagEstimatedPoseForInitialization()` |
| `RobotContainer.java` | Replaced `resetPoseFromAprilTagsOrPath2Start()` with `resetPoseFromChooser()` + `SendableChooser` |
| `Robot.java` | Updated call to `resetPoseFromChooser()` |

## New file

`src/test/java/frc/robot/subsystems/swervedrive/VisionPipelineTest.java` — 14 tests:

**computeStdDevs (6 tests):**
- No pose estimate → returns single-tag std devs
- Pose present but no recognized tags → returns single-tag std devs
- Single tag close (<4m) → returns scaled single-tag std devs
- Single tag far (>4m) → returns MAX_VALUE
- Multiple tags → returns scaled multi-tag std devs
- Multiple tags at zero distance → returns exact multi-tag std devs

**selectBestPose (8 tests):**
- All cameras empty → rejected as noEstimate
- Stale timestamp → rejected
- Single tag + high translation error → rejected as lowTagFar
- High std devs → rejected as highStd
- Outlier translation error → rejected
- Single valid camera → accepted with correct score
- Two valid cameras → lowest score wins
- Multi-tag bypasses single-tag distance filter
- Boundary values at 0.8m threshold (two tests)

## Build status

`./gradlew build` — all 28 tests pass, spotless clean.

## Still TODO

- Deploy to robot and verify `[VisionPipeline]` log lines appear in both teleop and auto
- Verify poses are still being fused correctly during autonomous
- Tune DriverStation alliance position coordinates for actual 2026 field
