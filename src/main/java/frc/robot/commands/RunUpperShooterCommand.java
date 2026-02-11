package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeShooterSubsystem;
import java.util.function.DoubleSupplier;

/**
 * RunUpperShooterCommand controls the upper CIM shooter motor based on trigger and bumper inputs.
 *
 * <p>Logic: 1. If left bumper is pressed → run motor CW (clockwise) at current speed setting 2.
 * Else if left trigger is pressed (≥ 0.9) → run motor CCW (counter-clockwise) at current speed
 * setting 3. Else → stop motor and apply brakes (0% output)
 *
 * <p>Note: The bumper takes priority over the trigger. If both are pressed, the bumper command (CW)
 * is executed.
 *
 * <p>Motor speed options (50%, 65%, 80%): - Current active speed: 50% (0.50) - To change to 65%:
 * Replace 0.50 with 0.65 in the SHOOTER_SPEED constant - To change to 80%: Replace 0.50 with 0.80
 * in the SHOOTER_SPEED constant
 */
public class RunUpperShooterCommand extends Command {

  private final IntakeShooterSubsystem intakeShooter;
  private final DoubleSupplier leftTriggerSupplier; // Left trigger axis (0.0 to 1.0)
  private final Trigger leftBumperTrigger; // Left bumper

  // Motor speed constant for the upper shooter
  // ===== SPEED OPTIONS (uncomment one, comment the others) =====
  private static final double SHOOTER_SPEED = 0.50; // Default: 50%

  // private static final double SHOOTER_SPEED = 0.65;  // Option: 65%
  // private static final double SHOOTER_SPEED = 0.80;  // Option: 80%

  /**
   * Constructs the RunUpperShooterCommand.
   *
   * @param intakeShooter the IntakeShooterSubsystem to control
   * @param leftTriggerSupplier provides the left trigger analog value (0.0 to 1.0)
   * @param leftBumperTrigger the left bumper Trigger
   */
  public RunUpperShooterCommand(
      IntakeShooterSubsystem intakeShooter,
      DoubleSupplier leftTriggerSupplier,
      Trigger leftBumperTrigger) {
    this.intakeShooter = intakeShooter;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.leftBumperTrigger = leftBumperTrigger;

    // Declare subsystem dependency
    addRequirements(intakeShooter);
  }

  @Override
  public void initialize() {
    // No special initialization needed; motor control happens in execute()
  }

  @Override
  public void execute() {
    // Get current input values
    double leftTrigger = leftTriggerSupplier.getAsDouble(); // 0.0 to 1.0
    boolean leftBumperPressed = leftBumperTrigger.getAsBoolean(); // true or false

    // Priority logic: bumper overrides trigger
    if (leftBumperPressed) {
      // Left bumper is pressed → run CW (negative output in FRC convention)
      // Negative output = clockwise rotation
      intakeShooter.setUpperMotor(-SHOOTER_SPEED);
    } else if (leftTrigger >= 0.9) {
      // Left trigger is fully pressed (≥ 0.9) → run CCW (positive output)
      // Positive output = counter-clockwise rotation
      intakeShooter.setUpperMotor(SHOOTER_SPEED);
    } else {
      // Neither input is active → stop motor and apply brakes
      // Setting output to 0.0 activates brakes due to NeutralMode.Brake configuration
      intakeShooter.setUpperMotor(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // When the command ends (e.g., when the trigger binding is released), stop the motor
    intakeShooter.stopAll();
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously while triggered (e.g., via .whileTrue() binding)
    // It does not finish on its own
    return false;
  }
}
