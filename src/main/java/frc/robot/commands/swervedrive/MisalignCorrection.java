package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class MisalignCorrection extends Command {
  private final SwerveSubsystem drivebase;
  private final SwerveDrive swerveDrive;
  private final String chassisDir;

  public MisalignCorrection(SwerveSubsystem drivebase, String chassisDirectory) {
    this.drivebase = drivebase;
    this.swerveDrive = drivebase.getSwerveDrive();
    this.chassisDir = chassisDirectory;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // SwerveModule[] modules = swerveDrive.getModules();
    double[] beforeAngles = new double[4];

    //DataLogManager.log("--- SWERVE CALIBRATION LEARN START ---");

    for (SwerveModule module : swerveDrive.getModules()) {
      // get module number for logging
      int i = module.moduleNumber;
      // 1. Capture current raw absolute position (what the encoder says right now)
      double currentPos = module.getAbsoluteEncoder().getAbsolutePosition();
      beforeAngles[i] = currentPos;

      // 2. Calculate the NEW offset.
      // We want the current physical position to represent '0' in the JSON.
      // If the wheel is straight but reading 10 degrees, the new offset should be 10.
      currentPos = module.getAbsolutePosition();

      // 3. Update the internal encoder wrapper's offset
      // This makes the change immediate in the software
      module.getAbsoluteEncoder().setAbsoluteEncoderOffset(currentPos);
    }

    // 4. Log the 'Before' state to AdvantageScope
    DataLogManager.log("Captured Offsets (Degrees): " + java.util.Arrays.toString(beforeAngles));
    DataLogManager.log(
        "Software offsets updated. REMINDER: You must still manually update JSONs later!");

    // 5. Re-sync the internal motor encoders to these new absolute zeros
    swerveDrive.synchronizeModuleEncoders();
  }

  @Override
  public void end(boolean interrupted) {}
}
