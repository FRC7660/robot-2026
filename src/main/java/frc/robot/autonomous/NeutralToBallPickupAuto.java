package frc.robot.autonomous;

import static frc.robot.Constants.NeutralToBallPickupAutoConstants.DEBUG_MODE_FORWARD_MPS;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.DEBUG_MODE_ROTATION_RAD_PER_SEC;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.DEFAULT_PICKUP_TIMEOUT_SEC;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.MAX_MODE_FORWARD_MPS;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.MAX_MODE_ROTATION_RAD_PER_SEC;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.OBJECT_APPROACH_MIN_FWD_MPS;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.OBJECT_AREA_TO_FWD_GAIN;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.OBJECT_PICKUP_AREA_PERCENT_THRESHOLD;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.OBJECT_YAW_TO_ROT_GAIN;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.PICKUP_PROXY_COUNT_DASHBOARD_KEY;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.POST_TIMEOUT_FINISH_GRACE_SEC;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.SEARCH_ROTATE_STEP_DEG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.IOException;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.json.simple.parser.ParseException;

public class NeutralToBallPickupAuto {
  public enum PickupDriveMode {
    MAX,
    DEBUG
  }

  private final SwerveSubsystem drivebase;
  private final String outboundTrajectoryName;
  private final String returnTrajectoryName;
  private final PickupDriveMode pickupDriveMode;
  private final double pickupTimeoutSec;

  public NeutralToBallPickupAuto(
      SwerveSubsystem drivebase, String outboundTrajectoryName, String returnTrajectoryName) {
    this(
        drivebase,
        outboundTrajectoryName,
        returnTrajectoryName,
        PickupDriveMode.DEBUG,
        DEFAULT_PICKUP_TIMEOUT_SEC);
  }

  public NeutralToBallPickupAuto(
      SwerveSubsystem drivebase,
      String outboundTrajectoryName,
      String returnTrajectoryName,
      PickupDriveMode pickupDriveMode,
      double pickupTimeoutSec) {
    this.drivebase = drivebase;
    this.outboundTrajectoryName = outboundTrajectoryName;
    this.returnTrajectoryName = returnTrajectoryName;
    this.pickupDriveMode = pickupDriveMode;
    this.pickupTimeoutSec = pickupTimeoutSec;
  }

  public Command build() {
    return Commands.sequence(
        Commands.print(
            String.format(
                "[NeutralToBallPickup] start outbound=%s pickup return=%s timeout=%.1fs mode=%s",
                outboundTrajectoryName, returnTrajectoryName, pickupTimeoutSec, pickupDriveMode)),
        followChoreoTrajectory(outboundTrajectoryName),
        pickupLoop(),
        followChoreoTrajectory(returnTrajectoryName),
        Commands.print("hands up -- shooting now"));
  }

  private Command followChoreoTrajectory(String trajectoryName) {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajectoryName);
            return AutoBuilder.followPath(path);
          } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError(
                String.format("[NeutralToBallPickup] Failed loading choreo '%s'", trajectoryName),
                e.getStackTrace());
            return Commands.none();
          }
        },
        Set.of(drivebase));
  }

  private Command pickupLoop() {
    AtomicInteger proxyPickups = new AtomicInteger(0);
    AtomicReference<Boolean> wasClose = new AtomicReference<>(false);
    AtomicReference<Double> lastHeadingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> rotateRemainingRad = new AtomicReference<>(0.0);
    AtomicReference<Double> lastLogSec = new AtomicReference<>(Double.NEGATIVE_INFINITY);
    AtomicReference<Double> startSec = new AtomicReference<>(0.0);
    AtomicReference<Boolean> allowApproachAfterTimeout = new AtomicReference<>(false);
    AtomicReference<Boolean> finished = new AtomicReference<>(false);

    return Commands.startRun(
            () -> {
              proxyPickups.set(0);
              wasClose.set(false);
              lastHeadingRad.set(drivebase.getHeading().getRadians());
              rotateRemainingRad.set(0.0);
              lastLogSec.set(Double.NEGATIVE_INFINITY);
              startSec.set(Timer.getFPGATimestamp());
              allowApproachAfterTimeout.set(false);
              finished.set(false);
              SmartDashboard.putNumber(PICKUP_PROXY_COUNT_DASHBOARD_KEY, 0);
              System.out.printf(
                  "[NeutralToBallPickup] pickup loop start timeout=%.1fs maxFwd=%.2f maxRot=%.2f%n",
                  pickupTimeoutSec, getMaxForwardMps(), getMaxRotationRadPerSec());
            },
            () -> {
              double headingNow = drivebase.getHeading().getRadians();
              double headingDelta =
                  Math.abs(MathUtil.angleModulus(headingNow - lastHeadingRad.get()));
              lastHeadingRad.set(headingNow);

              Optional<SwerveSubsystem.DetectedObjectObservation> observation =
                  drivebase.getBestDetectedObjectAnyCamera();
              boolean timedOut = (Timer.getFPGATimestamp() - startSec.get()) >= pickupTimeoutSec;
              double timePastTimeout =
                  Math.max(0.0, (Timer.getFPGATimestamp() - startSec.get()) - pickupTimeoutSec);

              if (observation.isPresent()) {
                double yawDeg = observation.get().yawDeg();
                double area = observation.get().area();

                double rotation =
                    MathUtil.clamp(
                        -Math.toRadians(yawDeg) * OBJECT_YAW_TO_ROT_GAIN,
                        -getMaxRotationRadPerSec(),
                        getMaxRotationRadPerSec());

                double forward = 0.0;
                if (area < OBJECT_PICKUP_AREA_PERCENT_THRESHOLD) {
                  forward =
                      MathUtil.clamp(
                          (OBJECT_PICKUP_AREA_PERCENT_THRESHOLD - area) * OBJECT_AREA_TO_FWD_GAIN,
                          OBJECT_APPROACH_MIN_FWD_MPS,
                          getMaxForwardMps());
                }

                drivebase.drive(new Translation2d(forward, 0), rotation, false);
                rotateRemainingRad.set(0.0);

                boolean closeNow = area >= OBJECT_PICKUP_AREA_PERCENT_THRESHOLD;
                if (closeNow && !wasClose.get()) {
                  proxyPickups.incrementAndGet();
                  SmartDashboard.putNumber(PICKUP_PROXY_COUNT_DASHBOARD_KEY, proxyPickups.get());
                }
                wasClose.set(closeNow);

                if (timedOut) {
                  allowApproachAfterTimeout.set(true);
                  if (closeNow || timePastTimeout >= POST_TIMEOUT_FINISH_GRACE_SEC) {
                    finished.set(true);
                  }
                }

                logPeriodic(
                    lastLogSec,
                    String.format(
                        "[NeutralToBallPickup] pickup tracking cam=%s yaw=%.2f area=%.2f fwd=%.2f rot=%.2f proxy=%d",
                        observation.get().cameraName(),
                        yawDeg,
                        area,
                        forward,
                        rotation,
                        proxyPickups.get()));
              } else {
                if (timedOut && allowApproachAfterTimeout.get()) {
                  finished.set(true);
                  drivebase.drive(new Translation2d(0, 0), 0, false);
                  return;
                }
                if (timedOut) {
                  finished.set(true);
                  drivebase.drive(new Translation2d(0, 0), 0, false);
                  return;
                }

                if (rotateRemainingRad.get() <= 0.0) {
                  rotateRemainingRad.set(Math.toRadians(SEARCH_ROTATE_STEP_DEG));
                }
                rotateRemainingRad.set(Math.max(0.0, rotateRemainingRad.get() - headingDelta));

                drivebase.drive(new Translation2d(0, 0), getMaxRotationRadPerSec(), false);
                wasClose.set(false);

                logPeriodic(
                    lastLogSec,
                    String.format(
                        "[NeutralToBallPickup] pickup searching rotateRemainingDeg=%.1f proxy=%d",
                        Math.toDegrees(rotateRemainingRad.get()), proxyPickups.get()));
              }
            },
            drivebase)
        .until(finished::get)
        .finallyDo(
            () -> {
              drivebase.drive(new Translation2d(0, 0), 0, false);
              System.out.printf(
                  "[NeutralToBallPickup] pickup loop end proxyPickups=%d%n", proxyPickups.get());
            });
  }

  private void logPeriodic(AtomicReference<Double> lastLogSec, String message) {
    double now = Timer.getFPGATimestamp();
    if (now - lastLogSec.get() >= 0.25) {
      lastLogSec.set(now);
      System.out.println(message);
    }
  }

  private double getMaxForwardMps() {
    return switch (pickupDriveMode) {
      case MAX -> MAX_MODE_FORWARD_MPS;
      case DEBUG -> DEBUG_MODE_FORWARD_MPS;
    };
  }

  private double getMaxRotationRadPerSec() {
    return switch (pickupDriveMode) {
      case MAX -> MAX_MODE_ROTATION_RAD_PER_SEC;
      case DEBUG -> DEBUG_MODE_ROTATION_RAD_PER_SEC;
    };
  }
}
