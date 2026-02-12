package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LowerShooterSubsystem;
import java.util.function.DoubleSupplier;

/**
 * RunLowerIntakeCommand controls the lower CIM intake motor based on trigger and bumper inputs.
 *
 * <p>Logic: 1. If right bumper is pressed → run motor CW (clockwise) at 75% output 2. Else if right
 * trigger is pressed (≥ 0.9) → run motor CCW (counter-clockwise) at 75% output 3. Else → stop motor
 * and apply brakes (0% output)
 *
 * <p>Note: The bumper takes priority over the trigger. If both are pressed, the bumper command (CW
 * at 75%) is executed.
 *
 * <p>Motor speed: Fixed at 75% (0.75 in percent mode).
 */
public class RunLowerIntakeCommand extends Command {

  private final LowerShooterSubsystem intakeShooter;
  private final DoubleSupplier rightTriggerSupplier; // Right trigger axis (0.0 to 1.0)
  private final Trigger rightBumperTrigger; // Right bumper

  /**
   * Constructs the RunLowerIntakeCommand.
   *
   * @param intakeShooter the IntakeShooterSubsystem to control
   * @param rightTriggerSupplier provides the right trigger analog value (0.0 to 1.0)
   * @param rightBumperTrigger the right bumper Trigger
   */
  public RunLowerIntakeCommand(
      LowerShooterSubsystem intakeShooter,
      DoubleSupplier rightTriggerSupplier,
      Trigger rightBumperTrigger) {
    this.intakeShooter = intakeShooter;
    this.rightTriggerSupplier = rightTriggerSupplier;
    this.rightBumperTrigger = rightBumperTrigger;

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
    double rightTrigger = rightTriggerSupplier.getAsDouble(); // 0.0 to 1.0
    boolean rightBumperPressed = rightBumperTrigger.getAsBoolean(); // true or false

    // Motor speed constant for the lower intake (75%)
    final double INTAKE_SPEED = 0.75;

    // Priority logic: bumper overrides trigger
    if (rightBumperPressed) {
      // Right bumper is pressed → run CW (negative output in FRC convention)
      // Negative output = clockwise rotation
      intakeShooter.setLowerMotor(-INTAKE_SPEED);
    } else if (rightTrigger >= 0.9) {
      // Right trigger is fully pressed (≥ 0.9) → run CCW (positive output)
      // Positive output = counter-clockwise rotation
      intakeShooter.setLowerMotor(INTAKE_SPEED);
    } else {
      // Neither input is active → stop motor and apply brakes
      // Setting output to 0.0 activates brakes due to NeutralMode.Brake configuration
      intakeShooter.setLowerMotor(0.0);
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
