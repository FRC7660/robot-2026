package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import swervelib.imu.SwerveIMU;

/**
 * YAGSL-compatible IMU that prefers NavX over MXP SPI and fails over to NavX USB1 when SPI
 * disconnects.
 */
public class DualNavxSwerveImu extends SwerveIMU {
  private enum ActiveSource {
    PRIMARY_SPI,
    SECONDARY_USB,
    NONE
  }

  private final MutAngularVelocity yawVel = new MutAngularVelocity(0, 0, DegreesPerSecond);
  private final AHRS primary;
  private final AHRS secondary;

  private Rotation3d offset = new Rotation3d();
  private boolean inverted = false;
  private ActiveSource activeSource = ActiveSource.NONE;
  private int failoverCount = 0;

  public DualNavxSwerveImu() {
    primary = tryCreate(AHRS.NavXComType.kMXP_SPI, "MXP SPI");
    secondary = tryCreate(AHRS.NavXComType.kUSB1, "USB1");
    updateStatus();
    factoryDefault();
  }

  @Override
  public void close() {
    if (primary != null) {
      primary.close();
    }
    if (secondary != null) {
      secondary.close();
    }
  }

  @Override
  public void factoryDefault() {
    offset = getRawRotation3d();
  }

  @Override
  public void clearStickyFaults() {}

  @Override
  public void setOffset(Rotation3d offset) {
    this.offset = offset;
  }

  @Override
  public void setInverted(boolean invertIMU) {
    inverted = invertIMU;
  }

  @Override
  public Rotation3d getRawRotation3d() {
    AHRS active = getActiveImu();
    if (active == null) {
      return new Rotation3d();
    }
    Rotation3d rotation = active.getRotation3d();
    return inverted ? negate(rotation) : rotation;
  }

  @Override
  public Rotation3d getRotation3d() {
    return getRawRotation3d().rotateBy(offset.unaryMinus());
  }

  @Override
  public Optional<Translation3d> getAccel() {
    AHRS active = getActiveImu();
    if (active == null) {
      return Optional.empty();
    }
    return Optional.of(
        new Translation3d(
                active.getWorldLinearAccelX(),
                active.getWorldLinearAccelY(),
                active.getWorldLinearAccelZ())
            .times(9.81));
  }

  @Override
  public MutAngularVelocity getYawAngularVelocity() {
    AHRS active = getActiveImu();
    return yawVel.mut_setMagnitude(active == null ? 0.0 : active.getRate());
  }

  @Override
  public Object getIMU() {
    return getActiveImu();
  }

  public synchronized void updateStatus() {
    ActiveSource previous = activeSource;
    if (isConnected(primary)) {
      activeSource = ActiveSource.PRIMARY_SPI;
    } else if (isConnected(secondary)) {
      activeSource = ActiveSource.SECONDARY_USB;
    } else {
      activeSource = ActiveSource.NONE;
    }
    if (previous != ActiveSource.SECONDARY_USB && activeSource == ActiveSource.SECONDARY_USB) {
      failoverCount++;
    }
  }

  public synchronized boolean isConnected() {
    updateStatus();
    return activeSource != ActiveSource.NONE;
  }

  public synchronized boolean primaryConnected() {
    return isConnected(primary);
  }

  public synchronized boolean secondaryConnected() {
    return isConnected(secondary);
  }

  public synchronized boolean isCalibrating() {
    AHRS active = getActiveImu();
    return active != null && active.isCalibrating();
  }

  public synchronized double getYaw() {
    AHRS active = getActiveImu();
    return active == null ? 0.0 : active.getYaw();
  }

  public synchronized int getFailoverCount() {
    return failoverCount;
  }

  public synchronized String getActiveSourceName() {
    updateStatus();
    return activeSource.name();
  }

  private synchronized AHRS getActiveImu() {
    updateStatus();
    return switch (activeSource) {
      case PRIMARY_SPI -> primary;
      case SECONDARY_USB -> secondary;
      case NONE -> null;
    };
  }

  private static Rotation3d negate(Rotation3d rotation) {
    return new Rotation3d(-rotation.getX(), -rotation.getY(), -rotation.getZ());
  }

  private static AHRS tryCreate(AHRS.NavXComType comType, String sourceLabel) {
    try {
      return new AHRS(comType);
    } catch (Throwable t) {
      DriverStation.reportWarning(
          "[NavX] Failed to initialize " + sourceLabel + " interface: " + t.getMessage(), false);
      return null;
    }
  }

  private static boolean isConnected(AHRS imu) {
    return imu != null && imu.isConnected();
  }
}
