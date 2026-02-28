// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BufferedLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  private double lastAutoPeriodicLogSec = Double.NEGATIVE_INFINITY;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Initialize the buffered disk logger — creates a new timestamped log file.
    BufferedLogger.getInstance();

    System.out.println("[RobotDebug] robotInit start");
    System.out.println("[RobotDebug] robotInit creating RobotContainer");
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    System.out.println("[RobotDebug] robotInit RobotContainer created");

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                System.out.printf(
                    "[CmdInit][%.2f][%s] %s%n",
                    Timer.getFPGATimestamp(),
                    DriverStation.isAutonomousEnabled() ? "AUTO" : "NONAUTO",
                    command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                System.out.printf(
                    "[CmdFinish][%.2f][%s] %s%n",
                    Timer.getFPGATimestamp(),
                    DriverStation.isAutonomousEnabled() ? "AUTO" : "NONAUTO",
                    command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                System.out.printf(
                    "[CmdInterrupt][%.2f][%s] %s%n",
                    Timer.getFPGATimestamp(),
                    DriverStation.isAutonomousEnabled() ? "AUTO" : "NONAUTO",
                    command.getName()));
    System.out.println("[RobotDebug] robotInit complete");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.printf("[RobotDebug][%.2f] disabledInit%n", Timer.getFPGATimestamp());
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.printf("[RobotDebug][%.2f] autonomousInit%n", Timer.getFPGATimestamp());
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.prepareAutonomous();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Print the selected autonomous command upon autonomous init
    System.out.println("Auto selected: " + m_autonomousCommand);

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println(
          "Auto schedule requested. isScheduled=" + m_autonomousCommand.isScheduled());
    } else {
      System.out.println("Auto schedule skipped because selected command was null.");
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double now = Timer.getFPGATimestamp();
    if (now - lastAutoPeriodicLogSec >= 1.0) {
      lastAutoPeriodicLogSec = now;
      String autoName = m_autonomousCommand == null ? "null" : m_autonomousCommand.getName();
      boolean scheduled = m_autonomousCommand != null && m_autonomousCommand.isScheduled();
      System.out.printf(
          "[RobotDebug][%.2f] autonomousPeriodic auto=%s scheduled=%s%n", now, autoName, scheduled);
    }
  }

  @Override
  public void teleopInit() {
    System.out.printf("[RobotDebug][%.2f] teleopInit%n", Timer.getFPGATimestamp());
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    System.out.printf("[RobotDebug][%.2f] testInit%n", Timer.getFPGATimestamp());
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
