package frc.robot.commands.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

// import frc.robot.Constants;

public class YAGSLPitCheck extends Command {
  // private final Subsystem subsystemOfSwerve;
  private final SwerveSubsystem drivebase;
  private final SwerveDrive swerveDrive;
  private final Timer timer = new Timer();
  private boolean stage1start = false;
  private String[] words = {"DONE", "...", "...", "..."};

  // Config: Adjust these to match your robot's gear ratios
  private final double STEER_GEAR_RATIO = 12.1; // Example for Mk4i

  public YAGSLPitCheck(SwerveSubsystem drivebase) {
    // this.subsystemOfSwerve = subsystemOfSwerve;
    this.drivebase = drivebase;
    this.swerveDrive = drivebase.getSwerveDrive();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    timer.restart();
    System.out.println("restarting pit check");
    stage1start = false;
  }

  public void start() {
    timer.reset();
    // stage1start = false;
  }

  @Override
  public void execute() {
    double elapsed = timer.get();

    // Stage 1: Align all wheels to 0 degrees (0 - 1.5s)
    if (elapsed < 1.5) {
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setAngle(90.0);
      }
      // drivebase.drive(new Translation2d(0, 1), 0, false);

    }

    // Stage 2: Spin Drive Motors at 10% (1.5s - 3s)
    else if (elapsed < 3.0) {
      swerveDrive.setMotorIdleMode(true); // Brake mode
      runHardwareSanityChecks();
      // YAGSL setRaw method to spin drive motors directly
      for (SwerveModule module : swerveDrive.getModules()) {
        if (module.moduleNumber == Math.floor(elapsed - 3.0)) { // counting from 0 to 3
          identificationDrive(module, 2);
        }
      }
    }

    // Stage 3: Spin Angle Motors consecutively at 10% (3s - 7s)
    else if (elapsed < 7.0) {
      for (SwerveModule module : swerveDrive.getModules()) {
        identificationTwirl(module, 0);
      }
      for (SwerveModule module : swerveDrive.getModules()) {
        if (module.moduleNumber == Math.floor(elapsed - 3.0)) { // counting from 0 to 3
          identificationTwirl(module, 2);
        }
      }
    }

    // Stage 4: Angle check
    else if (elapsed < 9.0) {
      for (SwerveModule module : swerveDrive.getModules()) {
        identificationTwirl(module, 0);
      }
      // dead period (allow the last wheel to realign from twirling)
    } else if (elapsed > 9.0) {
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setAngle(90.0);
      }
      swerveDrive.setMotorIdleMode(false);
      for (SwerveModule module : swerveDrive.getModules()) {
        module.getAngleMotor().setMotorBrake(false);
        alignmentCheck(module, elapsed);
      }
    }
  }

  private void alignmentCheck(SwerveModule module, Double time) {
    int number = module.moduleNumber;
    String name = "ModuleNum" + number;
    String path = "Diag/" + name + "/";
    boolean readError = (module.getAbsoluteEncoderReadIssue());
    boolean angleCorrect = (Math.abs(90 - module.getAbsolutePosition() % 180) < 4);
    if (readError) {
      SmartDashboard.putString(path + "Alignment Result", "READ ERROR");
    } else if (angleCorrect == false) {
      SmartDashboard.putString(
          path + "Alignment Result",
          "MISALIGNED: " + (Math.abs(90 - module.getAbsolutePosition() % 180)));
    } else {
      SmartDashboard.putString(path + "Alignment Result", "Aligned");
    }
    // SmartDashboard.putString(path, words[((int) Math.floor((number + time * -2)) % 4)]);
  }

  private void runHardwareSanityChecks() {
    for (SwerveModule module : swerveDrive.getModules()) {
      intelligentHelperFunction(module);
    }
  }

  private void identificationTwirl(SwerveModule module, int voltage) {
    int number = module.moduleNumber;
    String name = "ModuleNum" + number;
    String path = "Diag/" + name + "/";
    module.getAngleMotor().setVoltage(voltage);
    Double vel = module.getAngleMotor().getVelocity();
    if (Math.abs(vel) > 0) {
      SmartDashboard.putBoolean(path + "Twirl Complete", true);
    }
    SmartDashboard.putNumber(path + "Spin Velocity", vel);
  }

  private void identificationDrive(SwerveModule module, int voltage) {
    int number = module.moduleNumber;
    String name = "ModuleNum" + number;
    String path = "Diag/" + name + "/";
    module.getDriveMotor().setVoltage(voltage);
    Double vel = module.getDriveMotor().getVelocity();
    // if (Math.abs(vel) > 0) {
    //   SmartDashboard.putBoolean(path + "Twirl Complete", true);
    // }
    // SmartDashboard.putNumber(path + "Spin Velocity", vel);
  }

  private void intelligentHelperFunction(SwerveModule module) {
    int number = module.moduleNumber;
    String name = "ModuleNum" + number;
    String path = "Diag/" + name + "/";
    SmartDashboard.putBoolean(path + "Twirl Complete", false); // reset twirl condition
    SmartDashboard.putNumber(path + "Spin Velocity", 0);

    // 1. Get the TalonFX motor objects from YAGSL
    TalonFX driveKraken = (TalonFX) module.getDriveMotor().getMotor();
    TalonFX steerKraken = (TalonFX) module.getAngleMotor().getMotor();

    // 2. Connectivity Check
    boolean connected = driveKraken.getVelocity().getStatus().isOK();
    SmartDashboard.putBoolean(path + "CAN Connected", connected);

    // 3. Stator Current (Binding Check)
    double amps = driveKraken.getStatorCurrent().getValueAsDouble();
    SmartDashboard.putNumber(path + "Stator Amps", amps);

    // Check for mechanical binding (> 2A at only 20% output is a bad sign)
    SmartDashboard.putBoolean(path + "Drive Resistance OK", Math.abs(amps) < 2.0);

    // 4. Relative vs Absolute Consistency
    // Note: YAGSL handles the CANcoder, but we check the Kraken's internal rotor here
    double rotorPos = steerKraken.getRotorPosition().getValueAsDouble();
    double absolutePos = module.getAbsoluteEncoder().getAbsolutePosition(); // Degrees/Rotations

    // Verify if the internal rotor matches what the absolute encoder sees
    double expectedAbs = (rotorPos / STEER_GEAR_RATIO) % 1.0;
    boolean alignmentOK = Math.abs(expectedAbs - (absolutePos / 360.0)) < 0.05;
    // SmartDashboard.putBoolean(path + "Alignment OK", alignmentOK);

    // 5. Fault Reporting
    if (driveKraken.getStickyFault_Hardware(true).getValue()) {
      SmartDashboard.putString(path + "Hardware Status", "CRITICAL FAILURE");
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.setMotorIdleMode(false);
    for (SwerveModule module : swerveDrive.getModules()) {
      module.getAngleMotor().setMotorBrake(true);
    }
    drivebase.drive(new Translation2d(0, 0), 0, false);
  }
}
