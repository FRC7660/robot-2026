# Combining Two PhotonVision AprilTag Cameras with a Swerve Drive (WPILib)

You generally **don't choose one camera**. With PhotonVision + WPILib
you treat each camera as an independent pose sensor, convert each to a
**field-relative robot pose** using that camera's **robot↔camera
transform**, and then **fuse** them into your drivetrain pose estimator
(usually `SwerveDrivePoseEstimator`).

------------------------------------------------------------------------

## 1) Each camera must know where it is on the robot

You don't tell the swerve "which side" a camera is on. You define a
**fixed transform** from the **robot coordinate frame** to the
**camera**:

**WPILib Robot Frame** - +X = forward\
- +Y = left\
- +Z = up (right-handed)

For each camera create a `Transform3d robotToCam` with:

-   translation (meters): `(xForward, yLeft, zUp)`
-   rotation (radians): `(roll, pitch, yaw)`

That transform automatically handles "left vs right camera."

------------------------------------------------------------------------

## 2) For each camera: estimate robot pose on the field

Typical flow:

-   Load `AprilTagFieldLayout` (official field JSON)
-   Create a `PhotonPoseEstimator` per camera with:
    -   the field layout
    -   the camera name
    -   the `robotToCam` transform

Each loop:

-   Ask each estimator for an `EstimatedRobotPose`
-   Convert to `Pose2d`
-   Use the provided timestamp

------------------------------------------------------------------------

## 3) Fuse both cameras into `SwerveDrivePoseEstimator`

Run odometry continuously (gyro + modules), then add vision when
available:

``` java
poseEstimator.update(gyroYaw, modulePositions);
poseEstimator.addVisionMeasurement(visionPose2d, timestampSeconds);
```

If both cameras produce a measurement in the same cycle, you can add
both; the Kalman filter will blend them.

------------------------------------------------------------------------

## 4) Practical gating + weighting

Reject bad solves and weight good ones.

### Gate on:

-   Pose ambiguity
-   Tag count (multi-tag preferred)
-   Distance to tags
-   Timestamp freshness
-   Outlier vs current estimate

### Weighting:

-   Near tags + multi-tag → lower std devs (trust more)
-   Far tags + single tag → higher std devs (trust less)

You are not averaging poses manually---just giving the estimator
realistic uncertainty.

------------------------------------------------------------------------

## 5) Minimal structure

-   Define:
    -   `Transform3d robotToCamA`
    -   `Transform3d robotToCamB`
-   Create:
    -   `PhotonPoseEstimator camEstimatorA`
    -   `PhotonPoseEstimator camEstimatorB`
-   In periodic:
    -   update odometry
    -   for each estimator result:
        -   compute `Pose2d`
        -   compute `timestamp`
        -   choose std devs based on distance/tags
        -   `poseEstimator.addVisionMeasurement(...)`

------------------------------------------------------------------------

## 6) Common gotchas

-   Wrong camera rotation is the #1 issue (yaw/pitch/roll sign errors)
-   Ensure both cameras use the same field layout origin
-   Use PhotonVision's measurement timestamp when fusing
-   Let gating/weighting allow the better camera to dominate naturally

------------------------------------------------------------------------

If you provide: 1) Each camera's mounting position (x forward, y left, z
up in meters)\
2) Camera yaw/pitch/roll\
3) Whether PhotonVision multi-tag pose estimation is enabled

You can wire this directly into `SwerveDrivePoseEstimator` with tuned
gating and std-devs for FRC.
