package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeArm extends Command {

  private final Intake intake;

  public IntakeArm(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("Intake Arm Command Initialized");
    intake.runArm();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("arm in position");
    intake.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
