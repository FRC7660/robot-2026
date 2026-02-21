# Vision System Q&A

## Q: What does Vision.java do?

PhotonVision-based AprilTag vision system for correcting swerve drive odometry using camera pose estimation.

### Field Layout
- Loads the 2026 RebuiltWelded AprilTag field layout at class level, shared across all cameras.

### Camera Configuration (Cameras enum)
- Defines two active cameras: FRONT_CAMERA (`camera1`) and BACK_CAMERA (`camera0`), with a commented-out center camera (`camB`).
- Each camera stores:
  - Physical mounting position and rotation relative to robot center (`Transform3d`)
  - Single-tag and multi-tag standard deviations for confidence weighting
  - A `PhotonPoseEstimator` using MULTI_TAG_PNP_ON_COPROCESSOR strategy (fallback: LOWEST_AMBIGUITY)

### Pose Estimation Pipeline (`Vision.process()`)
1. `getCameraData()` - Fetches all unread results from each camera (real or simulated), sorts by timestamp.
2. `updateAprilTagError()` - Runs PhotonPoseEstimator on each camera's results and computes dynamic standard deviations.
3. `selectBestPose()` - Filters and scores candidates using the selected estimator mode (advanced or basic). Returns the best candidate.
4. `setVisionMeasurement()` - Applies the best candidate to the swerve drive pose estimator via `swerveDrive.addVisionMeasurement()`.

`process()` returns the `CameraSnapshot` map so other subsystems (e.g. FuelPalantir) can use the same camera data.

### Standard Deviation Heuristic (`computeStdDevs`)
- Multi-tag detections get tighter std devs than single-tag
- Single tag at >4m distance is rejected entirely (`Double.MAX_VALUE`)
- Std devs scale with `1 + (avgDist^2 / 30)`

### Advanced Pose Selection (`selectAdvancedPose`)
- Applies a multi-stage rejection pipeline:
  - Missing std devs - rejects candidates without valid standard deviations
  - Stale timestamp - skips already-fused results
  - Low tag count + far distance - rejects single-tag readings >0.8m from current pose
  - High std devs - rejects if X/Y std >5m or theta std >10rad
  - Outlier - rejects if translation error >1.5m from current odometry pose
- Scores candidates by: `(stdX + stdY) + 0.5*stdTheta + 0.25*translationError`

### Basic Pose Selection (`selectBasicPose`)
- Only rejects empty/stale estimates and candidates without valid standard deviations
- Picks the candidate with the lowest translation error

### Utility Methods
- `getAprilTagPose(id, offset)` - Returns a 2D pose relative to a specific AprilTag, with a robot offset transform applied
- `getDistanceFromAprilTag(id)` - Returns distance from current robot pose to a specific AprilTag
- `getTargetFromId(id, camera)` - Finds a specific tracked AprilTag target by fiducial ID from a camera's results
- `updateVisionField()` - Pushes all currently tracked AprilTag positions to the `Field2d` dashboard for visualization
- `getBestResult()` / `getLatestResult()` - Query the camera result cache by lowest ambiguity or most recent

### Simulation Support
- Creates a `VisionSystemSim` with simulated cameras (`PhotonCameraSim`) when running in simulation
- Simulated cameras: 960x720 resolution, 100-degree FOV, 30 FPS, ~35ms latency
- Updates sim with the actual drivetrain pose (not odometry) for accurate sim vision
- Posts vision estimate debug objects to the sim field

### Deprecated
- `filterPose()` - Old pose filtering by ambiguity threshold (0.25) and distance outlier rejection. Marked for removal since 2024, replaced by the pipeline in `process()`.
- `updatePoseEstimation()` - Old entry point, now delegates to `process()`.

---

## Q: What are the arguments to the Cameras enum constructor and what does each mean?

```java
Cameras(
    String name,                        // "camera1"
    Rotation3d robotToCamRotation,      // new Rotation3d(0, Math.toRadians(0), 0)
    Translation3d robotToCamTranslation,// new Translation3d(inches(0), inches(0), inches(12))
    Matrix<N3,N1> singleTagStdDevs,     // VecBuilder.fill(4, 4, 8)
    Matrix<N3,N1> multiTagStdDevsMatrix // VecBuilder.fill(0.5, 0.5, 1)
)
```

1. **`name`** - The PhotonVision camera name as it appears in the PhotonVision UI (e.g. `"camera1"`, `"camera0"`). This is used to create the `PhotonCamera` object that communicates with that specific camera over NetworkTables.

2. **`robotToCamRotation`** - A `Rotation3d` describing how the camera is rotated relative to the robot center. The three components are roll, pitch, yaw in radians. For example, `BACK_CAMERA` has `Math.toRadians(180)` yaw because it faces backward, and `Math.toRadians(5)` pitch because it's tilted slightly upward.

3. **`robotToCamTranslation`** - A `Translation3d` describing where the camera is physically mounted relative to the robot center, in meters (converted from inches). The three components are X (forward/back), Y (left/right), Z (up/down). For example, `FRONT_CAMERA` is 14 inches forward and 4 inches above center, while `BACK_CAMERA` is 14 inches behind center and 4 inches up.

4. **`singleTagStdDevs`** - A 3x1 matrix of standard deviations `[x, y, theta]` used when only one AprilTag is visible. Higher values = less trust in the measurement. The `(4, 4, 8)` means: 4m std dev in X, 4m in Y, 8 rad in theta -- so single-tag readings are treated with low confidence.

5. **`multiTagStdDevs`** - Same format but used when multiple AprilTags are visible simultaneously. The `(0.5, 0.5, 1)` values are much tighter, meaning multi-tag readings are trusted significantly more than single-tag ones.

These last two get further scaled dynamically by `updateEstimationStdDevs()` based on the average distance to visible tags.

---

## Q: singleTagStdDevs - standard deviation of what?

Standard deviation of the **estimated pose error** -- it represents how much you expect the vision-estimated robot position to be wrong when only one tag is visible.

The three values `(4, 4, 8)` map to:
- **4m** std dev in X position
- **4m** std dev in Y position
- **8 rad** std dev in heading (theta)

These get passed into `swerveDrive.addVisionMeasurement()`, which feeds into WPILib's **pose estimator** (a Kalman filter). The Kalman filter blends wheel odometry with vision measurements, and uses these std devs to decide **how much to trust each source**:

- **Small std dev** = "this measurement is precise, trust it heavily"
- **Large std dev** = "this measurement is noisy, don't shift the pose much"

So `(4, 4, 8)` for single-tag is saying "a single tag reading could easily be 4 meters off in position" -- which makes the Kalman filter mostly ignore it. Meanwhile `(0.5, 0.5, 1)` for multi-tag says "multi-tag readings are accurate to ~0.5 meters" -- so the filter trusts those much more.

In short: they're tuning knobs that control how aggressively vision corrections override wheel odometry.

---

## Q: How should we calculate these std devs?

There's no formula -- they're determined **empirically by testing on your actual robot**. Here's the practical approach:

### Method: Measure Ground Truth vs. Vision Estimates

1. Place the robot at a known position on the field (measure it precisely)
2. Record the vision-estimated pose over many frames from that position
3. Compute the actual standard deviation of the estimates vs. the known position
4. Repeat at varying distances from the tags (2m, 4m, 6m, etc.)
5. Repeat with 1 tag visible vs. multiple tags

You'll find that error grows with distance and shrinks with more tags -- which is exactly why the code already scales them dynamically in `updateEstimationStdDevs()`.

### What the code is already doing

The base values `(4, 4, 8)` and `(0.5, 0.5, 1)` are just starting points. The heuristic then scales them in real time:

```java
estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
```

So at 3m away, the multiplier is `1 + 9/30 = 1.3x`. At 6m, it's `1 + 36/30 = 2.2x`. This approximates the real-world behavior where error grows roughly quadratically with distance.

### Practical tuning approach

Rather than rigorously measuring, most FRC teams:

1. Start conservative (large std devs, like we have) so vision doesn't overcorrect
2. Drive the robot around during teleop near AprilTags and watch the pose estimate on the dashboard
3. If the pose jumps erratically when tags appear -- std devs are too small (trusting vision too much)
4. If the pose drifts and vision never corrects it -- std devs are too large (ignoring vision)
5. Tighten the values gradually until you get smooth, accurate corrections

### Key things to tune

- The base single vs. multi-tag values -- `(4,4,8)` vs `(0.5,0.5,1)`
- The distance scaling factor -- the `/30` divisor in the heuristic
- The single-tag cutoff distance -- currently 4m, beyond which single-tag is fully rejected
- Theta std dev relative to X/Y -- heading estimates from vision are typically less reliable, so theta should be larger

The values we have now are reasonable starting defaults. The real tuning happens on the practice field watching how the pose behaves during auto routines.

---

## Q: What is updateEstimationStdDevs and what is its purpose?

It dynamically adjusts the standard deviations for a camera each time new results come in, based on two factors: **how many tags are visible** and **how far away they are**.

The logic flows like this:

1. **No pose estimate** -- fall back to single-tag std devs (low trust)

2. **Pose exists but no valid tags found** -- same fallback

3. **One tag visible:**
   - If >4m away -- set std devs to `Double.MAX_VALUE` (effectively tells the Kalman filter to completely ignore this reading)
   - If <=4m -- use single-tag base values `(4, 4, 8)`, scaled by distance

4. **Multiple tags visible** -- use multi-tag base values `(0.5, 0.5, 1)`, scaled by distance

5. **Distance scaling** (applied in cases 3 and 4):
   ```java
   estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
   ```
   Computes the average distance to all visible tags, then inflates the std devs quadratically. Closer = tighter = more trust.

The result is stored in `curStdDevs`, which then gets read by `updatePoseEstimation()` when deciding whether to accept or reject the measurement and what confidence to pass to the Kalman filter.

In short: it's the "how much should we trust this particular camera reading right now" function.

---

## Q: How does updateEstimationStdDevs relate to the std devs on the camera enums?

The values on the camera enum (`singleTagStdDevs` and `multiTagStdDevs`) are the **base values** that `updateEstimationStdDevs()` starts from and then modifies.

The flow is:

1. Camera enum defines the starting points:
   - `singleTagStdDevs = (4, 4, 8)`
   - `multiTagStdDevs = (0.5, 0.5, 1)`

2. `updateEstimationStdDevs()` picks one of those based on tag count:
   ```java
   var estStdDevs = singleTagStdDevs;  // start with single
   if (numTags > 1) {
       estStdDevs = multiTagStdDevs;   // swap to multi if 2+ tags
   }
   ```

3. Then scales it by distance:
   ```java
   estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
   ```

4. Stores the final result in `curStdDevs`

So the enum values are never used directly by the Kalman filter -- they're always run through this heuristic first. The enum values are just the **per-camera tuning knobs** that the heuristic builds on.

Right now both cameras have identical base values `(4,4,8)` and `(0.5,0.5,1)`. But because they're defined per-camera, you could give different base confidence to each camera if, say, one had a better lens or a more stable mount.

---

## Q: Camera translation values -- what X, Y, Z should the camera enums be for our robot?

Our robot is a 28-inch square. The cameras are mounted at the base (4 inches off the ground) in the center of the front and back sides.

Using WPILib's robot coordinate system:
- **X** = forward/back (positive = toward front)
- **Y** = left/right (positive = toward left)
- **Z** = up/down (positive = up)

With a 28-inch square robot, the center is 14 inches from each edge.

**Front camera** (middle of front side):
```java
new Translation3d(
    Units.inchesToMeters(14),   // X: 14" forward from center
    Units.inchesToMeters(0),    // Y: centered
    Units.inchesToMeters(4))    // Z: 4" off the ground
```

**Back camera** (middle of back side):
```java
new Translation3d(
    Units.inchesToMeters(-14),  // X: 14" behind center
    Units.inchesToMeters(0),    // Y: centered
    Units.inchesToMeters(4))    // Z: 4" off the ground
```
