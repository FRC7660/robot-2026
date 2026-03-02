package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import java.util.Locale;

/** Default command for the turret: periodically compute a target and set a turret setpoint. */
public class TurretAutoTurn extends Command {
  private final Turret turret;
  private static final double PRINT_PERIOD_SEC = 0.5;
  private double lastPrintSec = 0.0;

  public TurretAutoTurn(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  public void initialize() {
    System.out.println("TurretAutoTurn initialized");

    lastPrintSec = 0.0; // reset the print timer
  }

  @Override
  public void execute() {
    Rotation2d robotRelative = turret.getRobotRelativeAngle();

    // Send the setpoint to the turret subsystem.
    turret.setTurretSetpoint(robotRelative);

    // Publish the setpoint for debugging.
    SmartDashboard.putNumber("Turret/SetpointRad", robotRelative.getRadians());
    // Also publish the chosen target from the turret for visibility
    var target = turret.getLastTarget();
    SmartDashboard.putNumber("Turret/TargetX", target.getX());
    SmartDashboard.putNumber("Turret/TargetY", target.getY());

    double now = Timer.getFPGATimestamp();
    if (now - lastPrintSec >= PRINT_PERIOD_SEC) {
      lastPrintSec = now;
      var pose = turret.getLastPose();
      var fieldAngle = turret.getLastFieldAngle();
      var turretAngle = turret.getLastSetpoint();
      double practicalDeg = turret.getLastRotationCommandDeg();
      double goalDeg = turret.getLastDesiredAbsDeg();
      String msg =
          String.format(
              Locale.US,
              "pose_x=%.2f, pose_y=%.2f, pose_deg=%.1f, robot_rel_deg=%.1f, field_rel_deg=%.1f, turret_deg=%.1f, practical_deg=%.1f, goal_deg=%.1f",
              pose.getX(),
              pose.getY(),
              pose.getRotation().getDegrees(),
              robotRelative.getDegrees(),
              fieldAngle.getDegrees(),
              turretAngle.getDegrees(),
              practicalDeg,
              goalDeg);
      DriverStation.reportWarning(msg, false);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Default command never finishes
  }

  @Override
  public void end(boolean isInterrupted) {
    turret.freeze(); // stop the motor
  }
}
