  Vision.java - Functionality Summary
  Core Purpose
  PhotonVision-based AprilTag vision system for correcting swerve drive odometry using camera pose estimation.
  Field Layout

  - Loads the 2026 RebuiltWelded AprilTag field layout at class level, shared across all cameras.

  Camera Configuration (Cameras enum)

  - Defines two active cameras: FRONT_CAMERA (camera1) and BACK_CAMERA (camera0), with a commented-out center camera (camB).
  - Each camera stores:
    - Physical mounting position & rotation relative to robot center (Transform3d)
    - Single-tag and multi-tag standard deviations for confidence weighting
    - A PhotonPoseEstimator using MULTI_TAG_PNP_ON_COPROCESSOR strategy (fallback: LOWEST_AMBIGUITY)

  Pose Estimation Pipeline

  1. updateUnreadResults() - Fetches all unread results from the camera (real or simulated), sorts by timestamp, and triggers pose estimation.
  2. updateEstimatedGlobalPose() - Iterates through cached results, runs the PhotonPoseEstimator, and updates standard deviations.
  3. updateEstimationStdDevs() - Dynamic standard deviation heuristic:
    - Multi-tag detections get tighter std devs than single-tag
    - Single tag at >4m distance is rejected entirely (Double.MAX_VALUE)
    - Std devs scale with 1 + (avgDist / 5)

  Vision Fusion (Autonomous Only)

  updatePoseEstimation() - The main entry point, called each loop. Only fuses vision during autonomous mode. It:
  - Iterates all cameras and selects the single best candidate per cycle
  - Applies a multi-stage rejection pipeline:
    - Stale timestamp - skips already-fused results
    - Low tag count + far distance - rejects single-tag readings >0.8m from current pose
    - High std devs - rejects if X/Y std >5m or theta std >10rad
    - Outlier - rejects if translation error >1.5m from current odometry pose
  - Scores candidates by: (stdX + stdY) + 0.5*stdTheta + 0.25*translationError
  - Calls swerveDrive.addVisionMeasurement() with the winning pose and its std devs
  - Logs detailed fusion diagnostics (accepted/rejected counts, per-update details) every 1 second
  - Prints an auto-period summary when autonomous ends

  Utility Methods

  - getAprilTagPose(id, offset) - Returns a 2D pose relative to a specific AprilTag, with a robot offset transform applied
  - getDistanceFromAprilTag(id) - Returns distance from current robot pose to a specific AprilTag
  - getTargetFromId(id, camera) - Finds a specific tracked AprilTag target by fiducial ID from a camera's results
  - updateVisionField() - Pushes all currently tracked AprilTag positions to the Field2d dashboard for visualization
  - getBestResult() / getLatestResult() - Query the camera result cache by lowest ambiguity or most recent

  Simulation Support

  - Creates a VisionSystemSim with simulated cameras (PhotonCameraSim) when running in simulation
  - Simulated cameras: 960x720 resolution, 100-degree FOV, 30 FPS, ~35ms latency
  - Updates sim with the actual drivetrain pose (not odometry) for accurate sim vision
  - Posts vision estimate debug objects to the sim field

  Deprecated

  - filterPose() - Old pose filtering by ambiguity threshold (0.25) and distance outlier rejection. Marked for removal since 2024, replaced by the fusion pipeline in
  updatePoseEstimation().



● Looking at the enum constructor (line 534) and an example instantiation (lines 445-451):

  Cameras(
      String name,                        // "camera1"
      Rotation3d robotToCamRotation,      // new Rotation3d(0, Math.toRadians(0), 0)
      Translation3d robotToCamTranslation,// new Translation3d(inches(0), inches(0), inches(12))
      Matrix<N3,N1> singleTagStdDevs,     // VecBuilder.fill(4, 4, 8)
      Matrix<N3,N1> multiTagStdDevsMatrix // VecBuilder.fill(0.5, 0.5, 1)
  )

  Here's what each one means:

  1. name - The PhotonVision camera name as it appears in the PhotonVision UI (e.g. "camera1", "camera0"). This is used to create the
  PhotonCamera object that communicates with that specific camera over NetworkTables.
  2. robotToCamRotation - A Rotation3d describing how the camera is rotated relative to the robot center. The three components are roll, pitch,
  yaw in radians. For example, BACK_CAMERA has Math.toRadians(180) yaw because it faces backward, and Math.toRadians(5) pitch because it's tilted
   slightly upward.
  3. robotToCamTranslation - A Translation3d describing where the camera is physically mounted relative to the robot center, in meters (converted
   from inches). The three components are X (forward/back), Y (left/right), Z (up/down). For example, FRONT_CAMERA is 12 inches above center,
  while BACK_CAMERA is 12 inches behind center.
  4. singleTagStdDevs - A 3x1 matrix of standard deviations [x, y, theta] used when only one AprilTag is visible. Higher values = less trust in
  the measurement. The (4, 4, 8) means: 4m std dev in X, 4m in Y, 8 rad in theta — so single-tag readings are treated with low confidence.
  5. multiTagStdDevs - Same format but used when multiple AprilTags are visible simultaneously. The (0.5, 0.5, 1) values are much tighter,
  meaning multi-tag readings are trusted significantly more than single-tag ones.

  These last two get further scaled dynamically by updateEstimationStdDevs() based on the average distance to visible tags.
