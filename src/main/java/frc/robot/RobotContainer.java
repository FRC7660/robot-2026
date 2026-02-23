// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutonomousManager;
import frc.robot.commands.turret.DefaultCommand;
import frc.robot.subsystems.LowerShooterSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.UpperShooterSubsystem;
import frc.robot.subsystems.swervedrive.FuelPalantir.FuelPalantirMode;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = createDrivebase();

  private final UpperShooterSubsystem upperShooter = new UpperShooterSubsystem();
  private final LowerShooterSubsystem lowerShooter = new LowerShooterSubsystem();

  // Turret subsystem, constructed with a supplier that returns the current odometry pose
  private final Turret turret = createTurret();

  private final AutonomousManager autonomousManager;

  private final SendableChooser<String> poseInitChooser;

  private SwerveSubsystem createDrivebase() {
    return new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/7660-jv0"));
  }

  private Turret createTurret() {
    return new Turret(drivebase::getPose);
  }

  private double getRightXCorrected() {
    double base = driverXbox.getRightX();
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        != DriverStation.Alliance.Red) {
      base *= -1;
    }
    return base;
  }

  private double getRightYCorrected() {
    double base = driverXbox.getRightY();
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        != DriverStation.Alliance.Red) {
      base *= -1;
    }
    return base;
  }

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(() -> getRightXCorrected(), () -> getRightYCorrected())
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand(
        "FuelPalantir", drivebase.fuelPalantirCommand(FuelPalantirMode.AUTONOMOUS));
    NamedCommands.registerCommand(
        "LogoFuelPalantir", drivebase.fuelPalantirCommand(FuelPalantirMode.AUTONOMOUS));
    NamedCommands.registerCommand(
        "ResetPoseFromAprilTags",
        Commands.runOnce(
            () -> {
              boolean reset = drivebase.resetOdometryFromAprilTags();
              System.out.printf("[PoseReset] source=APRILTAG commandResult=%s%n", reset);
            },
            drivebase));

    autonomousManager = new AutonomousManager(drivebase);

    poseInitChooser = new SendableChooser<>();
    poseInitChooser.setDefaultOption("PathPlanner path start", "pathplanner");
    poseInitChooser.addOption("DriverStation alliance position", "driverstation");
    poseInitChooser.addOption("Zero (origin)", "zero");
    SmartDashboard.putData("Pose Init", poseInitChooser);

    // Set the turret default command to compute targets from odometry
    turret.setDefaultCommand(new DefaultCommand(turret));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      driverXbox
          .b()
          .whileTrue(
              drivebase.driveToPose(
                  new Pose2d(new Translation2d(14, 3), Rotation2d.fromDegrees(180))));
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().whileTrue(drivebase.fuelPalantirCommand(FuelPalantirMode.TELEOP));
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      // JV bot shooter commands

      // INTAKE
      driverXbox
          .leftBumper()
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    upperShooter.setUpperMotor(0.85);
                    lowerShooter.setLowerMotor(-0.85);
                  },
                  () -> {
                    upperShooter.setUpperMotor(0);
                    lowerShooter.setLowerMotor(0);
                  },
                  upperShooter,
                  lowerShooter));

      // EXPEL(OUTTAKE)

      driverXbox
          .rightBumper()
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    upperShooter.setUpperMotor(-0.85);
                    lowerShooter.setLowerMotor(0.85);
                  },
                  () -> {
                    upperShooter.setUpperMotor(0);
                    lowerShooter.setLowerMotor(0);
                  },
                  upperShooter,
                  lowerShooter));

      // SHOOT
      driverXbox
          .leftTrigger(0.1)
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    upperShooter.setUpperMotor(0.85);
                    lowerShooter.setLowerMotor(0.85);
                  },
                  () -> {
                    upperShooter.setUpperMotor(0);
                    lowerShooter.setLowerMotor(0);
                  },
                  upperShooter,
                  lowerShooter));

      driverXbox
          .rightTrigger(0.1)
          .whileTrue(
              Commands.runEnd(
                  () -> {
                    upperShooter.setUpperMotor(driverXbox.getRightTriggerAxis());
                    lowerShooter.setLowerMotor(0.85);
                  },
                  () -> {
                    upperShooter.setUpperMotor(0);
                    lowerShooter.setLowerMotor(0);
                  },
                  upperShooter,
                  lowerShooter));

      // UNSTICK
      driverXbox
          .y()
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    upperShooter.setUpperMotor(-0.85);
                    lowerShooter.setLowerMotor(-0.85);
                  },
                  () -> {
                    upperShooter.setUpperMotor(0);
                    lowerShooter.setLowerMotor(0);
                  },
                  upperShooter,
                  lowerShooter));

      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(drivebase.fuelPalantirCommand(FuelPalantirMode.TELEOP));

      driverXbox.y().whileTrue(drivebase.sysIdDriveMotorCommand());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousManager.getAutonomousCommand();
  }

  public void resetPoseFromChooser() {
    String choice = poseInitChooser.getSelected();
    if (choice == null) {
      choice = "pathplanner";
    }

    switch (choice) {
      case "pathplanner":
        resetPoseFromPathPlanner();
        break;
      case "driverstation":
        resetPoseFromDriverStation();
        break;
      case "zero":
      default:
        drivebase.resetOdometry(new Pose2d());
        System.out.println("[PoseReset] source=ZERO pose=(0.000, 0.000, 0.0deg)");
        break;
    }
  }

  private void resetPoseFromPathPlanner() {
    try {
      // Use the selected auto file's first path to get the starting pose
      Command selectedAuto = autonomousManager.getAutonomousCommand();
      String autoName = selectedAuto != null ? selectedAuto.getName() : "path_to_center";
      String pathName = null;

      String selectedAutoName = autonomousManager.getSelectedAutoName();
      if (selectedAutoName != null) {
        pathName = getFirstPathNameFromAutoFile(selectedAutoName);
      }

      if (pathName == null) {
        // Extract path name from command name if possible, fall back to path_to_center
        pathName = "path_to_center";
        if (autoName.contains("PathPlanner-")) {
          int start = autoName.indexOf("PathPlanner-") + "PathPlanner-".length();
          String suffix = "-PathfindThenFollow";
          if (autoName.endsWith(suffix) && autoName.length() > start + suffix.length()) {
            pathName = autoName.substring(start, autoName.length() - suffix.length());
          } else {
            int end = autoName.indexOf("-", start);
            if (end > start) {
              pathName = autoName.substring(start, end);
            }
          }
        }
      }

      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Pose2d startPose = path.getStartingHolonomicPose().orElse(new Pose2d());
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        startPose = FlippingUtil.flipFieldPose(startPose);
      }
      drivebase.resetOdometry(startPose);
      System.out.printf(
          "[PoseReset] source=PATHPLANNER path=%s pose=(%.3f, %.3f, %.1fdeg)%n",
          pathName, startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees());
    } catch (IOException | ParseException | FileVersionException e) {
      drivebase.resetOdometry(new Pose2d());
      System.out.println(
          "[PoseReset] source=PATHPLANNER_FALLBACK_ZERO pose=(0.000, 0.000, 0.0deg)");
    }
  }

  private String getFirstPathNameFromAutoFile(String autoFileName) {
    File autoFile =
        new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoFileName + ".auto");
    if (!autoFile.exists()) {
      System.out.printf("[PoseReset] auto file not found: %s%n", autoFile.getAbsolutePath());
      return null;
    }

    try (FileReader reader = new FileReader(autoFile)) {
      Object parsed = new JSONParser().parse(reader);
      if (!(parsed instanceof JSONObject)) {
        return null;
      }
      JSONObject root = (JSONObject) parsed;
      JSONObject command = (JSONObject) root.get("command");
      String pathName = findFirstPathName(command);
      if (pathName != null) {
        System.out.printf("[PoseReset] auto=%s firstPath=%s%n", autoFileName, pathName);
      }
      return pathName;
    } catch (IOException | ParseException e) {
      DriverStation.reportWarning(
          "[PoseReset] Failed to parse PathPlanner auto '" + autoFileName + "': " + e.getMessage(),
          false);
      return null;
    }
  }

  private String findFirstPathName(JSONObject commandNode) {
    if (commandNode == null) {
      return null;
    }

    String type = (String) commandNode.get("type");
    JSONObject data = (JSONObject) commandNode.get("data");
    if ("path".equals(type) && data != null) {
      Object pathName = data.get("pathName");
      return pathName instanceof String ? (String) pathName : null;
    }

    if (data == null) {
      return null;
    }

    Object commandsObj = data.get("commands");
    if (commandsObj instanceof JSONArray) {
      JSONArray commands = (JSONArray) commandsObj;
      for (Object command : commands) {
        if (command instanceof JSONObject) {
          String pathName = findFirstPathName((JSONObject) command);
          if (pathName != null) {
            return pathName;
          }
        }
      }
    }

    Object nestedObj = data.get("command");
    if (nestedObj instanceof JSONObject) {
      return findFirstPathName((JSONObject) nestedObj);
    }

    return null;
  }

  private void resetPoseFromDriverStation() {
    // PLACEHOLDER positions -- measure on the actual 2026 field and update!
    // Blue alliance stations are on the left side, Red on the right.
    // Station 1 is closest to the scoring table, Station 3 is farthest.
    double[][] bluePositions = {
      {1.0, 1.0, 0.0}, // Station 1 (placeholder)
      {1.0, 4.0, 0.0}, // Station 2 (placeholder)
      {1.0, 7.0, 0.0}, // Station 3 (placeholder)
    };
    double[][] redPositions = {
      {16.0, 1.0, 180.0}, // Station 1 (placeholder)
      {16.0, 4.0, 180.0}, // Station 2 (placeholder)
      {16.0, 7.0, 180.0}, // Station 3 (placeholder)
    };

    var alliance = DriverStation.getAlliance();
    var location = DriverStation.getLocation();

    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    int station = location.isPresent() ? location.getAsInt() : 2; // default to station 2
    station = (int) MathUtil.clamp(station, 1, 3);

    double[][] positions = isRed ? redPositions : bluePositions;
    double[] pos = positions[station - 1];
    Pose2d startPose =
        new Pose2d(new Translation2d(pos[0], pos[1]), Rotation2d.fromDegrees(pos[2]));
    drivebase.resetOdometry(startPose);
    System.out.printf(
        "[PoseReset] source=DRIVERSTATION alliance=%s station=%d pose=(%.3f, %.3f, %.1fdeg)%n",
        isRed ? "Red" : "Blue",
        station,
        startPose.getX(),
        startPose.getY(),
        startPose.getRotation().getDegrees());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
