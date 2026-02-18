package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.ResetMode;
// import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IntakeShooterSubsystem controls two CIM brushed motors via Talon SRX motor controllers in brake
 * mode.
 *
 * <p>Motor Configuration: - Lower CIM (Intake): Talon SRX CAN ID 21 - Upper CIM (Shooter): Talon
 * SRX CAN ID 20
 *
 * <p>Both motors are set to BRAKE mode at startup (i.e., when power is lost or the motor is
 * commanded to 0, the motor holds its position rather than coasting).
 */
public class UpperShooterSubsystem extends SubsystemBase {

  private SparkMaxConfig configShooter;

  // private SparkMaxSim motorShooterSim;
  // private double desiredSpeed;

  private SparkMax upperMotor = new SparkMax(20, MotorType.kBrushless);

  // Talon SRX motor controllers
  // private final TalonSRX lowerMotor; // Intake motor (CAN ID 21)
  // private final TalonSRX upperMotor; // Shooter motor (CAN ID 20)

  /** Constructs the IntakeShooterSubsystem and configures brake mode at startup. */
  public UpperShooterSubsystem() {
    // Initialize the Talon SRX controllers with their CAN IDs
    // lowerMotor = new TalonSRX(21);
    // upperMotor = new TalonSRX(20);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake) // Set brake mode at startup
        .inverted(true); // Invert the shooter motor direction
    // configShooter = new SparkMaxConfig();
    // configShooter.idleMode(IdleMode.kBrake);
    // configShooter.inverted(true);
    upperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Set BRAKE mode for both motors at startup.
    // This means when the motor output is set to 0, the motor will actively brake
    // (hold position) instead of coasting to a stop. This is done once and does not change.
    // lowerMotor.setNeutralMode(NeutralMode.Brake);
    // upperMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Set the lower intake motor output.
   *
   * @param output Percent output from -1.0 (full reverse/CW) to 1.0 (full forward/CCW). 0.0 stops
   *     and applies brakes.
   */
  // public void setLowerMotor(double output) {
  //   // Clamp output to [-1.0, 1.0] to prevent invalid values
  //   output = Math.max(-1.0, Math.min(1.0, output));
  //   // Send the output command to the motor controller in PercentOutput mode
  //   lowerMotor.set(ControlMode.PercentOutput, output);
  // }

  /**
   * Set the upper shooter motor output.
   *
   * @param output Percent output from -1.0 (full reverse/CW) to 1.0 (full forward/CCW). 0.0 stops
   *     and applies brakes.
   */
  public void setUpperMotor(double output) {
    // Clamp output to [-1.0, 1.0] to prevent invalid values
    // output = Math.max(-1.0, Math.min(1.0, output));
    // Send the output command to the motor controller in PercentOutput mode
    // upperMotor.set(ControlMode.PercentOutput, output);
    upperMotor.set(output);
  }

  /** Stop both motors (set to 0% output, which applies brakes due to NeutralMode.Brake). */
  public void stopAll() {
    // setLowerMotor(0.0);
    setUpperMotor(0.0);
  }

  @Override
  public void periodic() {
    // No default behavior; control is entirely command-driven via setLowerMotor() and
    // setUpperMotor() calls from the command classes.
  }
}
