package frc.robot.commands.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PitCheckConstants;
import frc.robot.lib.DashboardTelemetry;
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
    String path = modulePath(module);
    boolean readError = (module.getAbsoluteEncoderReadIssue());
    boolean angleCorrect =
        (Math.abs(module.getAbsolutePosition() % 180) < PitCheckConstants.ANGLE_ENCODER_TOLERANCE);
    if (readError) {
      DashboardTelemetry.putString(path + "AlignmentResult", "READ ERROR");
    } else if (angleCorrect == false) {
      DashboardTelemetry.putString(
          path + "AlignmentResult",
          "MISALIGNED: " + (Math.abs(module.getAbsolutePosition() % 180)));
    } else {
      DashboardTelemetry.putString(path + "AlignmentResult", "Aligned");
    }
    DashboardTelemetry.putNumber(path + "AbsoluteEncoderValue", module.getAbsolutePosition());
    // DashboardTelemetry.putString(path, words[((int) Math.floor((number + time*-2)) % 4)]);
  }

  private void runHardwareSanityChecks() {
    for (SwerveModule module : swerveDrive.getModules()) {
      intelligentHelperFunction(module);
    }
  }

  private void identificationTwirl(SwerveModule module, double voltage) {
    String path = modulePath(module);
    module.getAngleMotor().setVoltage(voltage);
    Double vel = module.getAngleMotor().getVelocity();
    if (Math.abs(vel) > 0) {
      DashboardTelemetry.putBoolean(path + "TwirlComplete", true);
    }
    DashboardTelemetry.putNumber(path + "SpinVelocity", vel);
  }

  private void identificationDrive(SwerveModule module, double voltage) {
    String path = modulePath(module);
    module.getDriveMotor().setVoltage(voltage);
    Double vel = module.getDriveMotor().getVelocity();
    if (Math.abs(vel) > 0) {
      DashboardTelemetry.putBoolean(path + "DriveComplete", true);
    }
    DashboardTelemetry.putNumber(path + "DriveVelocity", vel);
  }

  private void intelligentHelperFunction(SwerveModule module) {
    String path = modulePath(module);
    DashboardTelemetry.putBoolean(path + "TwirlComplete", false); // reset twirl condition
    DashboardTelemetry.putNumber(path + "SpinVelocity", 0);

    // 1. Get the TalonFX motor objects from YAGSL
    TalonFX driveKraken = (TalonFX) module.getDriveMotor().getMotor();
    TalonFX steerKraken = (TalonFX) module.getAngleMotor().getMotor();

    // 2. Connectivity Check
    boolean connected = driveKraken.getVelocity().getStatus().isOK();
    DashboardTelemetry.putBoolean(path + "CanConnected", connected);

    // 3. Stator Current (Binding Check)
    double amps = driveKraken.getStatorCurrent().getValueAsDouble();
    DashboardTelemetry.putNumber(path + "StatorAmps", amps);

    // Check for mechanical binding (> 2A at only 20% output is a bad sign)
    DashboardTelemetry.putBoolean(
        path + "DriveResistanceOk", Math.abs(amps) < PitCheckConstants.STATOR_AMPS_THRESHOLD);

    // 4. Relative vs Absolute Consistency
    // Note: YAGSL handles the CANcoder, but we check the Kraken's internal rotor here
    double rotorPos = steerKraken.getRotorPosition().getValueAsDouble();
    double absolutePos = module.getAbsoluteEncoder().getAbsolutePosition(); // Degrees/Rotations

    // Verify if the internal rotor matches what the absolute encoder sees
    double expectedAbs = (rotorPos / PitCheckConstants.STEER_GEAR_RATIO) % 1.0;
    boolean alignmentOK =
        Math.abs(expectedAbs - (absolutePos / 360.0)) < PitCheckConstants.ALIGNMENT_ANGLE_TOLERANCE;
    // DashboardTelemetry.putBoolean(path + "AlignmentOk", alignmentOK);

    // 5. Fault Reporting
    if (driveKraken.getStickyFault_Hardware(true).getValue()) {
      DashboardTelemetry.putString(path + "HardwareStatus", "CRITICAL FAILURE");
    }
  }

  private String modulePath(SwerveModule module) {
    return String.format("PitCheck/Module%02d/", module.moduleNumber + 1);
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
