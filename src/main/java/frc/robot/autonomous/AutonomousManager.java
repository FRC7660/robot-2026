package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Set;

public class AutonomousManager {
  private static final String AUTO_TOGO = "togo";
  private static final String AUTO_THERE_BACK = "there-back";
  private static final String AUTO_NAME_PREFIX = "PathPlannerAuto-";

  private final SendableChooser<Command> autoChooser;
  private final Command togoAuto;
  private final Command thereBackAuto;
  private final SwerveSubsystem drivebase;

  public AutonomousManager(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    autoChooser = AutoBuilder.buildAutoChooser();
    togoAuto = buildPathPlannerAutoCommand(AUTO_TOGO);
    thereBackAuto = buildPathPlannerAutoCommand(AUTO_THERE_BACK);
    autoChooser.setDefaultOption("ThereBack: there-back", thereBackAuto);
    autoChooser.addOption("ToGo: togo", togoAuto);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    System.out.println("[AutoChooser] Selected command: " + selected);
    return selected == null ? thereBackAuto : selected;
  }

  /**
   * Prepare autonomous without forcing an extra odometry reset.
   *
   * <p>PathPlanner autos already honor each auto file's reset policy (`resetOdom`), so we avoid a
   * separate pre-auto reset path to keep one source of truth.
   */
  public void prepareAutonomousStart() {
    String selectedAutoName = getSelectedAutoName();
    System.out.printf(
        "[AutoChooser] prepareAutonomousStart selectedAuto=%s%n",
        selectedAutoName == null ? "unknown" : selectedAutoName);
  }

  public String getSelectedAutoName() {
    Command selected = autoChooser.getSelected();
    Command effective = selected == null ? thereBackAuto : selected;
    String name = effective.getName();
    if (name != null && name.startsWith(AUTO_NAME_PREFIX)) {
      return name.substring(AUTO_NAME_PREFIX.length());
    }
    return null;
  }

  private Command buildPathPlannerAutoCommand(String autoName) {
    return Commands.defer(
        () -> {
          try {
            System.out.println("[AutoChooser] Loading PathPlanner auto: " + autoName);
            return drivebase
                .getAutonomousCommand(autoName)
                .beforeStarting(
                    () ->
                        System.out.println(
                            "[AutoChooser] Starting PathPlanner auto command: " + autoName))
                .finallyDo(
                    interrupted ->
                        System.out.println(
                            "[AutoChooser] Finished PathPlanner auto command: "
                                + autoName
                                + " interrupted="
                                + interrupted))
                .withName(AUTO_NAME_PREFIX + autoName);
          } catch (RuntimeException e) {
            DriverStation.reportError(
                "[AutoChooser] Failed to load PathPlanner auto '" + autoName + "'",
                e.getStackTrace());
            return Commands.none();
          }
        },
        Set.of(drivebase));
  }
}
