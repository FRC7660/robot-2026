package frc.robot.lib;

/**
 * Helper that regulates turret movement around the zero point.
 *
 * <p>Behavior (per team pseudocode): - Maintains a persistent latch state when turret is "stuck"
 * at zero. - If latched and target remains within a sticky band, commands zero motion. -
 * Otherwise, releases latch and commands normal shortest-path motion.
 */
public final class TurretZeroPoint {
  private final double zeroAngleDeg;
  private final double stickyBandDeg;
  private final double zeroDeadbandDeg;

  // persistent latch state
  private boolean isStuckAtZero = false;

  public TurretZeroPoint() {
    this(0.0, 20.0, 1.0);
  }

  public TurretZeroPoint(double zeroAngleDeg, double stickyBandDeg, double zeroDeadbandDeg) {
    this.zeroAngleDeg = zeroAngleDeg;
    this.stickyBandDeg = stickyBandDeg;
    this.zeroDeadbandDeg = zeroDeadbandDeg;
  }

  /** Angular error between -180 and +180 degrees (signed). Works with degrees 0..359. */
  public static double angularErrorDeg(double targetDeg, double currentDeg) {
    double error = ((targetDeg - currentDeg + 180.0) % 360.0) - 180.0;
    // Java % can be negative, normalize
    if (error <= -180.0) {
      error += 360.0;
    } else if (error > 180.0) {
      error -= 360.0;
    }
    return error;
  }

  /**
   * Update the internal latch state and return a rotation command in degrees (signed). rotation
   * command indicates the signed angle delta the controller should move (degrees).
   *
   * @param targetAngleDeg desired target angle in degrees [0,360)
   * @param currentAngleDeg current turret angle in degrees [0,360)
   * @return rotation command in degrees (signed, within [-180,180])
   */
  public double updateAndCompute(double targetAngleDeg, double currentAngleDeg) {
    double error = angularErrorDeg(targetAngleDeg, currentAngleDeg);

    // Determine how close the target is to the configured zero angle. The "sticky" behavior
    // should latch based on how close the TARGET is to zero (not how close the target is to
    // the current position).
    double targetToZeroDeg = Math.abs(angularErrorDeg(zeroAngleDeg, targetAngleDeg));

    if (isStuckAtZero) {
      if (targetToZeroDeg <= stickyBandDeg) {
        // stay latched at zero
        return 0.0;
      }
      // release latch and command to move toward target (shortest path)
      isStuckAtZero = false;
      return error;
    }
    // Not currently latched at zero: decide if we should command to the zero even before
    // arriving there. The team rule: if the raw ideal angle (target) is within the sticky
    // band AND zero lies on the shortest path from current->target, then command toward
    // the zero (so the turret will prefer the zero route).
    double errorToZero = angularErrorDeg(zeroAngleDeg, currentAngleDeg);
    double errorToTarget = error;

    boolean zeroOnShortestPath =
        Math.signum(errorToZero) == Math.signum(errorToTarget)
            && Math.abs(errorToZero) <= Math.abs(errorToTarget);

    if (targetToZeroDeg <= stickyBandDeg && zeroOnShortestPath) {
      // Command to the zero (signed shortest delta to zero)
      return errorToZero;
    }

    // Not latching yet: check if we've arrived at zero and should set the persistent latch.
    double nearZero = Math.abs(errorToZero);
    boolean isNearZero = nearZero <= zeroDeadbandDeg;
    if (isNearZero && targetToZeroDeg <= stickyBandDeg) {
      // Arrived at zero while the target is within the sticky band: latch here.
      isStuckAtZero = true;
      return 0.0;
    }

    // Normal behavior: take the shortest path to the target.
    return error;
  }

  public boolean isStuckAtZero() {
    return isStuckAtZero;
  }

  public void setStuckAtZero(boolean stuck) {
    this.isStuckAtZero = stuck;
  }
}
