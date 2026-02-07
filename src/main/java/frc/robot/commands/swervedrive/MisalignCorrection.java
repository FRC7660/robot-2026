package frc.robot.commands.swervedrive;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PitCheckConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
    SwerveModule[] modules = drivebase.getSwerveDrive().getModules();

    for (SwerveModule module : modules) {
      JsonNode node = null;
      double pitCheckReading = Math.abs(90 - module.getAbsolutePosition() % 180);
      if (pitCheckReading > PitCheckConstants.ANGLE_ENCODER_TOLERANCE) {
        int moduleNumber = module.moduleNumber;
        String name = "ModuleNum" + moduleNumber;
        String SDpath = "Diag/" + name + "/";
        String[] posPaths = {"frontleft", "frontright", "backleft", "backright"};
        String posPath = posPaths[moduleNumber];
        ObjectMapper mapper = new ObjectMapper();
        String filePath =
            Filesystem.getDeployDirectory().toString()
                + File.separator
                + chassisDir
                + File.separator
                + "modules"
                + File.separator
                + posPath
                + ".json";
        try {
          node = mapper.readTree(new File(filePath));
        } catch (Exception e) {
          System.err.println("Failed to read JSON file: " + filePath);
          e.printStackTrace();
        }
        // extract the offset value from the JSON node and apply correction
        double offset = node.get("absoluteEncoderOffset").asDouble();
        double replacementOffset = (offset + module.getAbsolutePosition() + 90) % 360;
        ObjectNode newNode = (ObjectNode) node;
        newNode.put("absoluteEncoderOffset", replacementOffset);
        SmartDashboard.putNumber(
            SDpath + "JSON encoder value", newNode.get("absoluteEncoderOffset").asDouble());
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
