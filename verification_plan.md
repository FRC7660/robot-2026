# Real Robot Verification Plan

This verification plan covers the changes made (removing redundant inversion and fixing initialization) using the real robot.

Ideally, use the `Test` mode on the Driver Station or simply `Teleoperated` to run these checks. Ensure you have ample space and safety glasses.

### Test Protocol: Real Robot Verification

**Pre-Requisites:**
*   A "Forward" direction is defined on the floor (e.g., parallel to the side walls).
*   **Blue Forward:** Defined as driving AWAY from the Blue Driver Station.
*   **Red Forward:** Defined as driving AWAY from the Red Driver Station (which is driving Towards Blue).

---

### Test Case 1: Blue Alliance (No Vision)
**Objective:** Verify basic controls and non-inverted behavior.

1.  **Setup:**
    *   Set Driver Station to **Blue Alliance**.
    *   **Cover the Limelight/PhotonCamera** (or ensure no tags are visible).
    *   Place robot on field facing **AWAY** from the Driver Station (Field X+).
    *   Enable Robot.
    *   **Action:** Press the `'A'` button (Call `zeroGyroWithAlliance`).
        *   *Check:* Robot Heading should be ~0 degrees on Dashboard.

2.  **Steps:**
    *   **Drive Left Stick Forward:** Robot should move **Away** from you.
    *   **Drive Left Stick Right:** Robot should move **Right**.
    *   **Point Right Stick Right:** Robot should rotate to face **Right**.
    *   **Point Right Stick Forward:** Robot should rotate to face **Away**.

**Pass Criteria:** Robot movements match joystick inputs 1:1.

---

### Test Case 2: Red Alliance (No Vision) - *Critical Test*
**Objective:** Verify `SwerveInputStream` correctly handles inversion without manual interference.

1.  **Setup:**
    *   Set Driver Station to **Red Alliance**.
    *   **Cover the Limelight/PhotonCamera**.
    *   Place robot on field facing **AWAY** from the Driver Station (Towards Blue Wall).
    *   Enable Robot.
    *   **Action:** Press the `'A'` button.
        *   *Check:* Robot Heading should be **180 degrees** on Dashboard (or -180).
        *   *Note:* If it reads 0, the `zeroGyroWithAlliance` logic is failing to detect Red Alliance.

2.  **Steps:**
    *   **Drive Left Stick Forward:**
        *   *YAGSL Logic:* Inverts input (-X).
        *   *Robot Physics:* At 180°, moving -X (Field Backward) means moving Robot Forward.
        *   *Expectation:* Robot moves **AWAY** from you (Towards Blue).
    *   **Drive Left Stick Right:**
        *   *YAGSL Logic:* Inverts input (-Y).
        *   *Robot Physics:* At 180°, -Y (Field Right) is Robot Left.
        *   *Wait:* Field Right (from Blue perspective) is Red Driver's Left.
        *   *Expectation:* Robot moves **RIGHT** (Driver's Perspective).
    *   **Point Right Stick Right:**
        *   *Expectation:* Robot rotates to face **Driver's Right**.

**Pass Criteria:** Robot moves Away/Right relative to the *Driver*, even though the robot knows it is at 180°.

---

### Test Case 3: Blue Alliance (With Vision)
**Objective:** Verify Vision successfully overwrites the manual Zero.

1.  **Setup:**
    *   Set Driver Station to **Blue Alliance**.
    *   **Uncover Camera.**
    *   Place robot **skewed 45 degrees** to the left (facing somewhat diag-left), but with an AprilTag visible.
    *   Enable Robot.

2.  **Steps:**
    *   **Action:** Do *NOT* press 'A'. Wait 2 seconds.
    *   **Drive Left Stick Forward:**
        *   *Expectation:* Robot drives **Field Forward** (Away from Blue Wall, Parallel to sidelines), *not* Robot Forward (diagonal).
        *   *Why:* Vision should snap the field coordinates correctly, ignoring the fact you didn't manually zero.

---

### Test Case 4: Red Alliance (With Vision)
**Objective:** Verify Vision snap respects Red Alliance coordinate systems.

1.  **Setup:**
    *   Set Driver Station to **Red Alliance**.
    *   **Uncover Camera.**
    *   Place robot **skewed 45 degrees** relative to the Red Wall (facing somewhat diag-left away from driver), with AprilTag visible.
    *   Enable Robot.

2.  **Steps:**
    *   **Action:** Do *NOT* press 'A'. Wait 2 seconds.
    *   **Drive Left Stick Forward:**
        *   *Expectation:* Robot drives **Straight Away** from the driver (Field South / -X), correcting for the 45-degree skew.
    *   **Point Right Stick Forward:**
        *   *Expectation:* Robot rotates to face **Straight Away** from the driver (180° Field Heading).

**Pass Criteria:** Even without pressing 'A', the robot drives straight downfield because Vision determined the correct 180-ish starting pose.
