# ☘  FRC-7660: The Byting Irish's Varsity Robot 2026 ☘

This robot was built by Father Gabriel Richard Highschool's robotics team: The Byting Irish. It is quite a lively little fellow, and enjoys spinning around in its freetime. Learn more about our robotics team on [our website](https://fgrhs.org/byting-irish) and follow us on [our youtube channel](https://www.youtube.com/@frc7660) or on [our Instagram page](https://instagram.com/fgrrobotics)

![](https://yt3.googleusercontent.com/I9DuCqpA8V1Xk0CXAuu-DUtK5PPjO9vSilhLGOc0HHOFYFihNt9_PjpeIAPgGxdFGAFJOC2N=s160-c-k-c0x00ffffff-no-rj)

## NeutralToBallPickup Autonomous

### Summary
`NeutralToBallPickupAuto` is a hybrid autonomous routine that combines:
- Choreo split pathing for deterministic travel to/from the ball area
- Vision-guided pickup behavior during a timed middle phase

Current flow:
1. Follow Choreo split `0` from neutral zone to ball area
2. Run a timed pickup loop for `5s`
3. If timeout occurs while currently approaching an object, allow a short finish window
4. Follow Choreo split `1` for return
5. Print `"hands up -- shooting now"` as the end action placeholder

### Strategy
- Use Choreo for repeatable long-travel segments.
- Use object detection in the pickup zone where exact trajectories are less reliable.
- Keep search simple and deterministic:
  - If object exists, drive toward it.
  - If no object exists, rotate CCW in 15-degree search chunks.
- Use object area as a proxy for pickup:
  - Area crossing `8%` counts as a proxy pickup event.
- Return is timeout-driven, not count-driven.

### Architecture
- `frc.robot.autonomous.NeutralToBallPickupAuto`
  - Builds the full command sequence (`split0 -> pickup -> split1 -> shoot print`).
  - Loads Choreo splits using `PathPlannerPath.fromChoreoTrajectory(name, splitIndex)` and follows with `AutoBuilder.followPath(...)`.
  - Runs pickup loop with `Commands.startRun(...)`.
- `frc.robot.autonomous.AutonomousManager`
  - Registers this routine in the chooser as `Neutral To Ball Pickup`.
  - Sets this routine as the default autonomous option.
- `frc.robot.subsystems.swervedrive.SwerveSubsystem`
  - Provides `getBestDetectedObjectAnyCamera()` to expose the best non-fiducial object across camera0/camera1.
- `frc.robot.Constants.NeutralToBallPickupAutoConstants`
  - Central place for all routine tuning constants.

### Choreo Requirements
- Expected trajectory base name: `NeutralToBallPickup`
- Expected file path:
  - `src/main/deploy/choreo/NeutralToBallPickup.traj`
- Expected splits in this file:
  - Split `0`: outbound to ball area
  - Split `1`: return path

You do not need one file per split. One `.traj` with multiple splits is sufficient.

### SmartDashboard
- Proxy pickup count is published to:
  - `Auto/NeutralToBallPickup/ProxyPickups`

### Constants (Current Defaults)
Location: `frc.robot.Constants.NeutralToBallPickupAutoConstants`

- `DEFAULT_TRAJECTORY_NAME = "NeutralToBallPickup"`
- `DEFAULT_PICKUP_TIMEOUT_SEC = 5.0`
- `POST_TIMEOUT_FINISH_GRACE_SEC = 1.5`
- `SEARCH_ROTATE_STEP_DEG = 15.0` (CCW search)
- `OBJECT_PICKUP_AREA_PERCENT_THRESHOLD = 8.0`
- `OBJECT_APPROACH_MIN_FWD_MPS = 0.10`
- `OBJECT_AREA_TO_FWD_GAIN = 0.08`
- `OBJECT_YAW_TO_ROT_GAIN = 2.0`
- Drive mode caps:
  - `MAX_MODE_FORWARD_MPS = 0.35`
  - `MAX_MODE_ROTATION_RAD_PER_SEC = 0.35`
  - `DEBUG_MODE_FORWARD_MPS = 0.35`
  - `DEBUG_MODE_ROTATION_RAD_PER_SEC = 0.35`

### Notes on AprilTag Usage
- Choreo path following itself does not require AprilTags.
- The drivetrain pose estimate can still be corrected by vision if enabled in the swerve subsystem.
- In this project, vision-assisted updates remain enabled during autonomous/pickup behavior.
