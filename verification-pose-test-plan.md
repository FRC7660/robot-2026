# Verification Test Plan: PathPlanner Starting Pose Fix

This plan covers end-to-end verification of the two pose-reset bug fixes:
1. **Alliance mirroring** — path2 start pose flipped correctly for Red alliance
2. **Pose reset timing** — reset moved from `robotInit()` to `autonomousInit()`

---

## Pre-requisites

- Robot deployed with the fixed code
- Driver Station connected and alliance color set correctly
- PathPlanner `path2.path` file present on the robot (deploy directory)
- SmartDashboard / Shuffleboard open to observe pose field widget

---

## Test 1 — Console log sanity check (both alliances)

**Goal:** Confirm `[PoseReset]` log line appears at auto init (not robot init) and contains correct coordinates.

### Steps
1. Connect DS, set **Blue** alliance.
2. Power cycle / reboot robot code.
3. In DS console, verify no `[PoseReset]` log line appears during `robotInit`.
4. Enable **Autonomous**.
5. Confirm a `[PoseReset]` log line appears immediately before auto starts.
6. Record the pose coordinates.

| Condition | Expected X (m) | Expected Y (m) | Notes |
|-----------|---------------|---------------|-------|
| Blue alliance, no tags visible | < 8.27 | any | Blue side of field |
| Red alliance, no tags visible  | > 8.27 | any | Red side (mirrored) |

7. Disable. Switch DS to **Red** alliance.
8. Enable Autonomous again.
9. Confirm `[PoseReset]` pose now shows Red-side coordinates.

**Pass criteria:** Log shows `source=PATH2_FALLBACK` with coordinates on the correct side of the field for each alliance.

---

## Test 2 — Field widget pose check (static, no driving)

**Goal:** Visually confirm odometry is reset to the right location on the field widget.

### Steps
1. Open Shuffleboard / SmartDashboard field widget showing robot pose.
2. Set **Blue** alliance, enable Autonomous, immediately disable (within ~0.5 s).
3. Observe robot icon on field widget — should be at path2's Blue starting position.
4. Repeat for **Red** alliance — robot icon should appear at path2's mirrored Red starting position.

**Pass criteria:** Robot icon lands on the correct starting waypoint for each alliance color.

---

## Test 3 — Full autonomous run (Blue alliance)

**Goal:** Robot follows path2 in the correct direction on Blue.

### Steps
1. Physically place robot at path2's Blue starting position and heading.
2. Set DS to **Blue** alliance.
3. Enable Autonomous.
4. Observe robot motion.

**Pass criteria:**
- Robot moves along path2's intended route without large deviations.
- `pathfindThenFollowPath` phase is minimal (robot is already at path start, so no large pathfinding detour).
- `[AutoChooser] Finished PathPlanner path command: path2 interrupted=false` appears in console.

---

## Test 4 — Full autonomous run (Red alliance)

**Goal:** Robot follows path2's mirrored path correctly on Red.

### Steps
1. Physically place robot at path2's Red (mirrored) starting position and heading.
2. Set DS to **Red** alliance.
3. Enable Autonomous.
4. Observe robot motion.

**Pass criteria:**
- Robot mirrors Test 3 behavior on the Red side of the field.
- No large initial pathfinding detour (robot starts in the correct place).

---

## Test 5 — AprilTag pose override (if tags visible)

**Goal:** Confirm AprilTag-sourced pose is still used when a tag is visible, and that it takes priority over path2 fallback.

### Steps
1. Position robot where at least one AprilTag is in view of CAMERA0 or CAMERA1.
2. Enable Autonomous.
3. Check DS console for `[PoseReset] source=APRILTAG`.
4. Verify the logged pose is plausible for the robot's actual location.

**Pass criteria:** `source=APRILTAG` appears (not `PATH2_FALLBACK`), and pose is close to actual robot location.

---

## Test 6 — Alliance not yet set edge case

**Goal:** Confirm no crash or bad pose reset if DS alliance is `Optional.empty()` (e.g., practice/test mode with no FMS).

### Steps
1. Disconnect from FMS (practice mode or no FMS).
2. Do not set alliance color in DS.
3. Enable Autonomous.
4. Confirm no exception/crash. Confirm `[PoseReset]` log appears.

**Pass criteria:** Robot initializes without error. Pose reset defaults to Blue side (fallback behavior when alliance is unknown).

---

## Regression Check

After all tests pass, verify teleop driving is unaffected:
- Switch to Teleop, drive the robot around.
- Confirm field-relative driving behaves normally for both alliances.
- Confirm `[PoseReset]` log does NOT appear during teleop transitions.
