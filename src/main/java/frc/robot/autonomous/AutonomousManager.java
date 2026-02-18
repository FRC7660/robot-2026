package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.IOException;
import java.util.Set;
import org.json.simple.parser.ParseException;

public class AutonomousManager {
  private final SendableChooser<Command> autoChooser;
  private final Command path2Auto;
  private final SwerveSubsystem drivebase;

  public AutonomousManager(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    autoChooser = AutoBuilder.buildAutoChooser();
    path2Auto = buildPathPlannerPath2Command();
    autoChooser.setDefaultOption("ToGo: path2", path2Auto);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    System.out.println("[AutoChooser] Selected command: " + selected);
    return selected == null ? path2Auto : selected;
  }

  private Command buildPathPlannerPath2Command() {
    return Commands.defer(
        () -> {
          try {
            final String pathName = "path2";
            System.out.println("[AutoChooser] Loading PathPlanner path: " + pathName);
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            System.out.println("[AutoChooser] Loaded PathPlanner path: " + pathName);

            PathConstraints constraints =
                new PathConstraints(
                    0.35,
                    0.35,
                    drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(),
                    Units.degreesToRadians(720.0));

            return AutoBuilder.pathfindThenFollowPath(path, constraints)
                .beforeStarting(
                    () ->
                        System.out.println(
                            "[AutoChooser] Starting PathPlanner path command: " + pathName))
                .finallyDo(
                    interrupted ->
                        System.out.println(
                            "[AutoChooser] Finished PathPlanner path command: "
                                + pathName
                                + " interrupted="
                                + interrupted))
                .withName("PathPlanner-" + pathName + "-PathfindThenFollow");
          } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError(
                "[AutoChooser] Failed to load PathPlanner path 'path2'", e.getStackTrace());
            return Commands.none();
          }
        },
        Set.of(drivebase));
  }
}
