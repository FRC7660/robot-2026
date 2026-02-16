package frc.robot.commands.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PitCheckConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class YAGSLPitCheck extends Command {
  private final SwerveSubsystem drivebase;
  private final SwerveDrive swerveDrive;
  private final Timer timer = new Timer();

  // private boolean stage1start = false;
  // private String[] words = {"DONE", "...", "...", "..."};

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
    // stage1start = false;
  }

  public void start() {
    timer.reset();
  }

  @Override
  public void execute() {
    double elapsed = timer.get();

    // Stage 1: Align all wheels to 0 degrees (0 - 1.5s)
    if (elapsed < 1.5) {
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setAngle(0.0);
      }
      // drivebase.drive(new Translation2d(0, 1), 0, false);

    }

    // Stage 2: Spin Drive Motors at 10% (1.5s - 3s)
    else if (elapsed < 3.0) {
      swerveDrive.setMotorIdleMode(true); // Brake mode
      runHardwareSanityChecks();
      // YAGSL setRaw method to spin drive motors directly
      for (SwerveModule module : swerveDrive.getModules()) {
        if (module.moduleNumber
            == Math.floor((elapsed - 1.5) * 2.666)) { // counting from 0 to 3 over 1.5s
          identificationDrive(module, PitCheckConstants.MOTOR_TEST_VOLTAGE);
        }
      }
    }

    // Stage 3: Spin Angle Motors consecutively at 10% (3s - 7s)
    else if (elapsed < 7.0) {
      for (SwerveModule module : swerveDrive.getModules()) {
        identificationDrive(module, 0);
      }
      for (SwerveModule module : swerveDrive.getModules()) {
        if (module.moduleNumber == Math.floor(elapsed - 3.0)) { // counting from 0 to 3
          identificationTwirl(module, PitCheckConstants.MOTOR_TEST_VOLTAGE);
        }
      }
    }

    // Stage 4: Realign wheels to 0 degrees (7s - 9s)
    else if (elapsed < 9.0) {
      // Actively control angle back to 0 degrees using PID
      for (SwerveModule module : swerveDrive.getModules()) {
        module.setAngle(0.0);
      }
    }

    // Stage 5: Final alignment check with motors in coast mode (9s+)
    else if (elapsed > 9.0) {
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
    boolean angleCorrect =
        (Math.abs(module.getAbsolutePosition() % 180) < PitCheckConstants.ANGLE_ENCODER_TOLERANCE);
    if (readError) {
      SmartDashboard.putString(path + "Alignment Result", "READ ERROR");
    } else if (angleCorrect == false) {
      SmartDashboard.putString(
          path + "Alignment Result",
          "MISALIGNED: " + (Math.abs(module.getAbsolutePosition() % 180)));
    } else {
      SmartDashboard.putString(path + "Alignment Result", "Aligned");
    }
    SmartDashboard.putNumber(path + "Absolute Encoder Value", module.getAbsolutePosition());
    // SmartDashboard.putString(path, words[((int) Math.floor((number + time*-2)) % 4)]);
  }

  private void runHardwareSanityChecks() {
    for (SwerveModule module : swerveDrive.getModules()) {
      intelligentHelperFunction(module);
    }
  }

  private void identificationTwirl(SwerveModule module, double voltage) {
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

  private void identificationDrive(SwerveModule module, double voltage) {
    int number = module.moduleNumber;
    String name = "ModuleNum" + number;
    String path = "Diag/" + name + "/";
    module.getDriveMotor().setVoltage(voltage);
    Double vel = module.getDriveMotor().getVelocity();
    if (Math.abs(vel) > 0) {
      SmartDashboard.putBoolean(path + "Drive Complete", true);
    }
    SmartDashboard.putNumber(path + "Drive Velocity", vel);
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
    SmartDashboard.putBoolean(
        path + "Drive Resistance OK", Math.abs(amps) < PitCheckConstants.STATOR_AMPS_THRESHOLD);

    // 4. Relative vs Absolute Consistency
    // Note: YAGSL handles the CANcoder, but we check the Kraken's internal rotor here
    double rotorPos = steerKraken.getRotorPosition().getValueAsDouble();
    double absolutePos = module.getAbsoluteEncoder().getAbsolutePosition(); // Degrees/Rotations

    // Verify if the internal rotor matches what the absolute encoder sees
    double expectedAbs = (rotorPos / PitCheckConstants.STEER_GEAR_RATIO) % 1.0;
    boolean alignmentOK =
        Math.abs(expectedAbs - (absolutePos / 360.0)) < PitCheckConstants.ALIGNMENT_ANGLE_TOLERANCE;
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
