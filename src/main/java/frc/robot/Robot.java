// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // UTF-8 sentinel comment: 🤖☘️

  private static Robot instance;
  private final Map<String, Integer> commandCounts = new HashMap<>();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

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
    Logger.recordMetadata("ProjectName", "robot-2026");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("RobotControllerSerialNumber", RobotController.getSerialNumber());

    switch (Constants.CURRENT_MODE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter("_sim"));
        break;
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(
        command ->
            Logger.recordOutput(
                "CommandsUnique/"
                    + command.getName()
                    + "_"
                    + Integer.toHexString(command.hashCode()),
                true));
    scheduler.onCommandFinish(
        command ->
            Logger.recordOutput(
                "CommandsUnique/"
                    + command.getName()
                    + "_"
                    + Integer.toHexString(command.hashCode()),
                false));
    scheduler.onCommandInterrupt(
        command ->
            Logger.recordOutput(
                "CommandsUnique/"
                    + command.getName()
                    + "_"
                    + Integer.toHexString(command.hashCode()),
                false));

    scheduler.onCommandInitialize(
        command -> {
          int count = commandCounts.getOrDefault(command.getName(), 0) + 1;
          commandCounts.put(command.getName(), count);
          Logger.recordOutput("CommandsAll/" + command.getName(), count > 0);
        });
    scheduler.onCommandFinish(
        command -> {
          int count = commandCounts.getOrDefault(command.getName(), 0) - 1;
          if (count <= 0) {
            commandCounts.remove(command.getName());
            Logger.recordOutput("CommandsAll/" + command.getName(), false);
          } else {
            commandCounts.put(command.getName(), count);
          }
        });
    scheduler.onCommandInterrupt(
        command -> {
          int count = commandCounts.getOrDefault(command.getName(), 0) - 1;
          if (count <= 0) {
            commandCounts.remove(command.getName());
            Logger.recordOutput("CommandsAll/" + command.getName(), false);
          } else {
            commandCounts.put(command.getName(), count);
          }
        });
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
    Logger.recordOutput("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.onDisable();
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
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.autonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
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
