// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutonomousManager;
import frc.robot.commands.swervedrive.MisalignCorrection;
import frc.robot.commands.swervedrive.YAGSLPitCheck;
import frc.robot.lib.BufferedLogger;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDsystem.Lights;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.FuelPalantir.FuelPalantirMode;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;
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
  private final CommandGenericHID buttonBox = new CommandGenericHID(1);
  private final Intake intakeSystem = new Intake();

  // The robot's subsystems and commands are defined here...
  private final String chassisDirectory = "swerve/7660-chassis1";
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), chassisDirectory));
  private final MisalignCorrection misalignCorrection =
      new MisalignCorrection(drivebase, chassisDirectory);
  private final Index indexSystem = new Index();
  // Turret subsystem, constructed with a supplier that returns the current odometry pose
  private final Turret turret = new Turret(drivebase::getPose);
  // Launch subsystem
  private final Launch launchSystem = new Launch();
  private boolean imuFallbackActive = false;

  // LED system
  private final Lights lights = new Lights(launchSystem, intakeSystem, drivebase, turret);

  private final AutonomousManager autonomousManager;

  private double getRightXCorrected() {
    if (RobotBase.isSimulation()) {
      return driverXbox.getRawAxis(3) * -1;
    }
    double base = driverXbox.getRightX();
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        != DriverStation.Alliance.Red) {
      base *= -1;
    }
    return base;
  }

  private double getRightYCorrected() {
    if (RobotBase.isSimulation()) {
      return driverXbox.getRawAxis(4) * -1;
    }
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
          .scaleTranslation(1.0)
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
          .scaleTranslation(1.0)
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
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    BufferedLogger.getInstance();

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    try {
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
      // New commands for comp
      // NamedCommands.registerCommand(
      //    "startAutoAim",
      // turret.autoSetAngle().repeatedly().andThen(Commands.run(turret::freeze)));
      NamedCommands.registerCommand(
          "startShootingSequence", launchSystem.shotSequenceStartWithTurret(indexSystem, turret));
      NamedCommands.registerCommand("armOut&Running", intakeSystem.fullIntake());
      NamedCommands.registerCommand("armIn", intakeSystem.retract());
    } catch (Exception e) {
      DriverStation.reportError(
          "[NamedCommands] registration failed: " + e.getMessage(), e.getStackTrace());
    }

    autonomousManager = new AutonomousManager(drivebase);
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
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    // Intake speed init
    DoubleSupplier leftTriggerSupplier =
        () -> {
          return driverXbox.getLeftTriggerAxis();
        };
    Command runIntakeWithSpeed =
        Commands.run(() -> intakeSystem.setRollerSpeed(leftTriggerSupplier.getAsDouble()));
    Command stopIntake = Commands.run(() -> intakeSystem.stopRoller());

    // Default drive style
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // IMU fault protection (Teleop only): if the IMU disconnects, force angular-velocity drive so
    // the driver retains direct rotational control instead of the heading PID spinning
    // uncontrollably.
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled() && !drivebase.navxConnected() && !imuFallbackActive)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                  imuFallbackActive = true;
                }));

    // Teleop-only recovery: when IMU communication returns, restore direct-angle drive mode.
    new Trigger(
            () -> DriverStation.isTeleopEnabled() && drivebase.navxConnected() && imuFallbackActive)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
                  imuFallbackActive = false;
                }));

    // SHOOTING CONTROL
    // Trigger (CLASS) which will initiate trigger (INPUT) control of the launch and turret
    Trigger shotPressureDetected = new Trigger(() -> (driverXbox.getRightTriggerAxis() > 0.25));
    Trigger shotPressureMaxed = new Trigger(() -> (driverXbox.getRightTriggerAxis() > 0.85));

    // Shot sequence init (Binds to Right Trigger)
    Command startSequence = launchSystem.shotSequenceStart(indexSystem, turret);
    startSequence.addRequirements(launchSystem, indexSystem);
    // Right Trigger/DPad Up - Medium pressure: start AutoTurn
    shotPressureDetected.whileTrue(turret.autoSetAngleThenFreeze());
    driverXbox.povUp().whileTrue(turret.autoSetAngleThenFreeze());
    // Right Trigger - Full pressure: start shooting and indexing sequence
    shotPressureMaxed.whileTrue(startSequence);
    // Right Bumper - Force-start shooting and indexing (no aim)
    driverXbox.rightBumper().whileTrue(startSequence);

    // INTAKE CONTROL
    // Toggle the arm out/in
    Trigger intakeSingleTap = driverXbox.leftBumper().multiPress(1, 0.25);
    intakeSingleTap.onTrue(intakeSystem.toggleIntake());
    Trigger intakeDoubleTap = driverXbox.leftBumper().multiPress(2, 0.25);
    intakeDoubleTap.onTrue(Commands.runOnce(() -> intakeSystem.setAngleSetpoint(70.0)));
    // Run and stop the intake based on the Left Trigger value (threshold/deadzone of 0.1)
    driverXbox.leftTrigger(0.1).whileTrue(runIntakeWithSpeed);
    driverXbox.leftTrigger(0.1).onFalse(stopIntake);

    // ZEROS
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.back().whileTrue(intakeSystem.zeroArm());
    driverXbox.povDown().onTrue(turret.zeroTurret());

    if (Robot.isSimulation()) {
      driverXbox.back().onTrue(Commands.runOnce(() -> drivebase.resetToStartingPosition()));
    }

    if (DriverStation.isTest()) {
      driverXbox.povDown().whileTrue(new YAGSLPitCheck(drivebase));
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

  public void autonomousInit() {
    autonomousManager.prepareAutonomousStart();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void onDisable() {
    drivebase.setMotorBrake(true);
    intakeSystem.setMotorBrake(false);
  }
}
