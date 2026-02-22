# Vision System Architecture Diagram

## File Structure

```
swervedrive/
  Vision.java                  -- Orchestrator: fusion logic, field layout, utilities
  Cameras.java                 -- Enum: per-camera config, raw pose estimation, std dev heuristic
  SwerveSubsystem.java         -- Drives the robot; reads cameras directly for targeting + object detection
  AprilTagBallShuttleAuto.java -- Autonomous routine; reads cameras directly for tag/object tracking
  PhotonObjectPrinter.java     -- Debug logger; reads cameras directly for object detection printing
```

## Top-Level Entry Points Into Camera Data

There are THREE independent paths that read from the cameras.
They do NOT go through a single gateway -- each reads `camera.camera.getLatestResult()`
or `camera.resultsList` directly.

```
                        PhotonVision Coprocessor
                                 |
                                 | (NetworkTables)
                                 v
                     +------ PhotonCamera ------+
                     |           |               |
          PATH 1     |    PATH 2 |        PATH 3 |
      (pose fusion)  |  (targeting)       (debug) |
                     |           |               |
                     v           v               v

            Vision.java   SwerveSubsystem.java   PhotonObjectPrinter.java
                          AprilTagBallShuttleAuto.java
```

### PATH 1: Pose Fusion (Vision.java)
- Called from: `SwerveSubsystem.periodic()` -> `vision.updatePoseEstimation()`
- Reads via: `Cameras.getEstimatedGlobalPose()` -> `camera.getAllUnreadResults()`
- Purpose: Correct odometry with AprilTag pose estimates
- Active: AUTO ONLY

### PATH 2: Direct Camera Reads for Targeting (SwerveSubsystem + AprilTagBallShuttleAuto)
- Called from: autonomous commands, `aimAtTarget()`, `trackDetectedObject()`, etc.
- Reads via: `camera.camera.getLatestResult()` (bypasses Vision.java entirely)
- Purpose: Real-time tag centering, object detection, approach commands
- Active: ANYTIME (auto + teleop)
- Callers include:
  - `SwerveSubsystem.aimAtTarget(camera)` -- uses `camera.getBestResult()`
  - `SwerveSubsystem.trackDetectedObject(camera)` -- uses `camera.camera.getLatestResult()`
  - `SwerveSubsystem.logDetectedObjectArea(camera)` -- uses `camera.camera.getLatestResult()`
  - `SwerveSubsystem.getBestDetectedObjectAnyCamera()` -- iterates CAMERA0/CAMERA1
  - `SwerveSubsystem.getVisibleAprilTagByIdOnCamera()` -- uses `camera.camera.getLatestResult()`
  - `SwerveSubsystem.getClosestVisibleAprilTagObservation()` -- iterates CAMERA0/CAMERA1
  - `SwerveSubsystem.setInitialPoseFromAprilTags()` -- uses `vision.getEstimatedGlobalPose(camera)`
  - `AprilTagBallShuttleAuto` -- same pattern, all direct `camera.camera.getLatestResult()`

### PATH 3: Debug Logging (PhotonObjectPrinter)
- Called from: periodic timer
- Reads via: `camera.getLatestResult()` (on the raw PhotonCamera)
- Purpose: Print detected object info to console at low rate

## PATH 1 Detail: Pose Fusion Call Flow

```
Robot periodic loop
  |
  v
SwerveSubsystem.periodic()
  |
  v
Vision.updatePoseEstimation(swerveDrive)        <-- ENTRY POINT
  |
  |-- [sim only] visionSim.update(truePose)      Update sim with real drivetrain pose
  |
  |-- [if not auto] return early                  Vision fusion is AUTO-ONLY
  |
  |-- for each Cameras enum value:
  |     |
  |     v
  |   Vision.getEstimatedGlobalPose(camera)       Wrapper with sim debug output
  |     |
  |     v
  |   Cameras.getEstimatedGlobalPose()            Public entry on each camera
  |     |
  |     v
  |   Cameras.updateUnreadResults()               Fetch + sort new PhotonVision frames
  |     |
  |     v
  |   Cameras.updateEstimatedGlobalPose()         Run PhotonPoseEstimator on each frame
  |     |
  |     v
  |   Cameras.updateEstimationStdDevs()           Heuristic: adjust curStdDevs
  |     |                                          based on tag count + distance
  |     v
  |   returns Optional<EstimatedRobotPose>
  |     + camera.curStdDevs is now set
  |
  |-- REJECTION PIPELINE (per candidate):
  |     |
  |     |-- reject if no estimate
  |     |-- reject if stale timestamp
  |     |-- reject if single-tag + far (>0.8m from odom)
  |     |-- reject if std devs too large (>5m XY or >10rad theta)
  |     |-- reject if translation outlier (>1.5m from odom)
  |     |
  |     v
  |   score = (stdX + stdY) + 0.5*stdTheta + 0.25*translationError
  |   keep lowest-scoring candidate
  |
  v
swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs)
  |
  v
WPILib Kalman Filter blends vision + wheel odometry
```

## PATH 2 Detail: Direct Camera Reads

```
SwerveSubsystem / AprilTagBallShuttleAuto
  |
  |-- These functions read cameras DIRECTLY, bypassing Vision.java:
  |
  |   aimAtTarget(camera)
  |     |-> camera.getBestResult()              Cached resultsList, lowest ambiguity
  |     |-> target.getYaw() -> rotate robot
  |
  |   trackDetectedObject(camera)
  |     |-> camera.camera.getLatestResult()     Raw PhotonCamera query
  |     |-> target.getYaw() -> rotate robot
  |
  |   getBestDetectedObjectAnyCamera()
  |     |-> for CAMERA0, CAMERA1:
  |     |     camera.camera.getLatestResult()   Raw PhotonCamera query
  |     |-> pick most centered non-fiducial target
  |
  |   getVisibleAprilTagByIdOnCamera(tagId, camera)
  |     |-> camera.camera.getLatestResult()     Raw PhotonCamera query
  |     |-> find target with matching fiducial ID
  |
  |   getClosestVisibleAprilTagObservation()
  |     |-> for CAMERA0, CAMERA1:
  |     |     camera.camera.getLatestResult()   Raw PhotonCamera query
  |     |-> pick tag with largest area (closest)
  |
  |   setInitialPoseFromAprilTags()
  |     |-> vision.getEstimatedGlobalPose(camera)  Uses PATH 1's pipeline
  |     |-> swerveDrive.resetOdometry(pose)
  |
  |   NOTE: These read from camera.camera (the raw PhotonCamera),
  |         NOT from camera.resultsList which is only populated
  |         when PATH 1 calls getEstimatedGlobalPose().
```

## Cameras.java Internal Pipeline

```
Cameras enum instance
  |
  |-- FRONT_CAMERA ("camera1")    x=+14", y=0, z=4"   faces forward
  |-- BACK_CAMERA  ("camera0")    x=-14", y=0, z=4"   faces backward, 5deg pitch
  |
  |   Each camera has:
  |     PhotonCamera           -- NetworkTables comms with coprocessor
  |     PhotonPoseEstimator    -- MULTI_TAG_PNP_ON_COPROCESSOR strategy
  |     singleTagStdDevs       -- base std devs: (4, 4, 8)
  |     multiTagStdDevs        -- base std devs: (0.5, 0.5, 1)
  |     curStdDevs             -- dynamically computed each frame
  |     resultsList            -- cached pipeline results (PATH 1 only)
  |     estimatedRobotPose     -- latest pose estimate (PATH 1 only)
  |
  |
  getEstimatedGlobalPose()           <-- only called by PATH 1
    |
    v
  updateUnreadResults()
    |
    |-- Fetch all unread results from PhotonCamera
    |-- Sort by timestamp
    |-- [optional] Log object detection debug info
    |
    v
  updateEstimatedGlobalPose()
    |
    |-- for each result in resultsList:
    |     poseEstimator.update(result)        PhotonVision solves pose
    |     updateEstimationStdDevs(pose)       Adjust confidence
    |
    v
  updateEstimationStdDevs(estimatedPose, targets)
    |
    |-- No pose?           --> curStdDevs = singleTagStdDevs
    |-- No valid tags?     --> curStdDevs = singleTagStdDevs
    |-- 1 tag, >4m away?   --> curStdDevs = MAX_VALUE (ignore)
    |-- 1 tag, <=4m?       --> curStdDevs = singleTagStdDevs * distanceScale
    |-- 2+ tags?           --> curStdDevs = multiTagStdDevs  * distanceScale
    |
    |   distanceScale = 1 + (avgDist / 5)
    |
    v
  curStdDevs is set --> read by Vision.updatePoseEstimation()
```

## Vision.java Utility Functions

```
Vision
  |
  |-- getAprilTagPose(tagId, offset)      Static. Look up tag pose, apply offset.
  |                                       Used for autonomous target positioning.
  |
  |-- getDistanceFromAprilTag(tagId)      Distance from current robot pose to a tag.
  |
  |-- getTargetFromId(tagId, camera)      Find a specific tracked target by fiducial ID
  |                                       from a camera's result cache (resultsList).
  |
  |-- updateVisionField()                 Push tracked tag positions to Field2d dashboard.
  |                                       Reads resultsList from all cameras.
  |
  |-- getVisionSim()                      Returns VisionSystemSim (sim only).
  |
  |-- openSimCameraViews()                [sim only] Would open browser to camera streams.
  |                                       Currently commented out.
  |
  |-- filterPose()                        @Deprecated. Old ambiguity-based filter.
  |                                       Replaced by rejection pipeline in
  |                                       updatePoseEstimation().
```

## Cameras.java Query Functions

```
Cameras
  |
  |-- getBestResult()       Returns cached result with lowest pose ambiguity.
  |                         Not necessarily the most recent.
  |                         Only works if PATH 1 has populated resultsList.
  |
  |-- getLatestResult()     Returns most recent cached result from resultsList.
  |                         Only works if PATH 1 has populated resultsList.
  |
  |-- addToVisionSim()      [sim only] Register camera in VisionSystemSim.
```

## End-to-End Data Flow Summary

```
PhotonVision Coprocessor
        |
        | (NetworkTables)
        v
  PhotonCamera  ----+-----------------------------+--------------------+
        |           |                             |                    |
        |      (PATH 2+3)                    (PATH 1)                  |
        |   .getLatestResult()          .getAllUnreadResults()          |
        |   Direct, real-time           Batched, sorted              |
        |           |                             |                    |
        v           v                             v                    v
   SwerveSubsystem     AprilTagBallShuttleAuto     Cameras.resultsList
   targeting/objects    targeting/objects                |
                                                       v
                                              PhotonPoseEstimator.update()
                                                       |
                                                       v
                                              updateEstimationStdDevs()
                                                       |
                                                       v
                                              Vision.updatePoseEstimation()
                                                rejection pipeline
                                                       |
                                                       v
                                              SwerveDrive.addVisionMeasurement()
                                                       |
                                                       v
                                              WPILib Kalman Filter
                                                       |
                                                       v
                                              SwerveDrive.getPose()
                                              (corrected robot pose)
```
