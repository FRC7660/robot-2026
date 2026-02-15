package frc.robot.autonomous;

import static frc.robot.Constants.NeutralToBallPickupAutoConstants.DEFAULT_OUTBOUND_TRAJECTORY_NAME;
import static frc.robot.Constants.NeutralToBallPickupAutoConstants.DEFAULT_RETURN_TRAJECTORY_NAME;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.AprilTagBallShuttleAuto;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Set;
import org.json.simple.parser.ParseException;

public class AutonomousManager {
  private final SendableChooser<Command> autoChooser;
  private final Command shuttleAuto;
  private final Command neutralToBallPickupAuto;
  private final SwerveSubsystem drivebase;

  public AutonomousManager(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    shuttleAuto = new AprilTagBallShuttleAuto(drivebase).build(5);
    neutralToBallPickupAuto =
        new NeutralToBallPickupAuto(
                drivebase, DEFAULT_OUTBOUND_TRAJECTORY_NAME, DEFAULT_RETURN_TRAJECTORY_NAME)
            .build();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Neutral To Ball Pickup", neutralToBallPickupAuto);
    autoChooser.addOption("AprilTag Ball Shuttle x5", shuttleAuto);
    autoChooser.addOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    addChoreoOptions();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    System.out.println("[AutoChooser] Selected command: " + selected);
    return selected == null ? shuttleAuto : selected;
  }

  private void addChoreoOptions() {
    File choreoDir = new File(Filesystem.getDeployDirectory(), "choreo");
    File[] trajFiles = choreoDir.listFiles((dir, name) -> name.endsWith(".traj"));

    if (trajFiles == null || trajFiles.length == 0) {
      System.out.println("[AutoChooser] No Choreo trajectories found in deploy/choreo");
      return;
    }

    Arrays.sort(trajFiles, Comparator.comparing(File::getName));
    for (File trajFile : trajFiles) {
      String filename = trajFile.getName();
      String trajName = filename.substring(0, filename.length() - ".traj".length());
      autoChooser.addOption("Choreo: " + trajName, buildChoreoCommand(trajName));
    }
  }

  private Command buildChoreoCommand(String trajectoryName) {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(trajectoryName);
            return AutoBuilder.followPath(choreoPath);
          } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError(
                "[AutoChooser] Failed to load Choreo trajectory '" + trajectoryName + "'",
                e.getStackTrace());
            return Commands.none();
          }
        },
        Set.of(drivebase));
  }
}
