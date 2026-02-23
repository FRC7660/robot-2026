package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/** Default command for the turret: periodically compute a target and set a turret setpoint. */
public class DefaultCommand extends Command {
  private final Turret turret;

  public DefaultCommand(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Rotation2d robotRelative = turret.getRobotRelativeAngle();

    // Send the setpoint to the turret subsystem (stubbed implementation).
    turret.setTurretSetpoint(robotRelative);

    // Publish the setpoint for debugging.
    SmartDashboard.putNumber("Turret/SetpointRad", robotRelative.getRadians());
    // Also publish the chosen target from the turret for visibility
    var target = turret.getLastTarget();
    SmartDashboard.putNumber("Turret/TargetX", target.getX());
    SmartDashboard.putNumber("Turret/TargetY", target.getY());
  }

  @Override
  public boolean isFinished() {
    return false; // Default command never finishes
  }
}
