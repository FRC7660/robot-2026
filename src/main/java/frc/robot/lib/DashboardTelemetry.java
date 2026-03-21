package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public final class DashboardTelemetry {
  private DashboardTelemetry() {}

  public static boolean isHighVerbosity() {
    return Constants.Telemetry.isHighVerbosity();
  }

  public static void putBoolean(String key, boolean value) {
    Logger.recordOutput(key, value);
    if (isHighVerbosity()) {
      SmartDashboard.putBoolean(key, value);
    }
  }

  public static void putNumber(String key, double value) {
    Logger.recordOutput(key, value);
    if (isHighVerbosity()) {
      SmartDashboard.putNumber(key, value);
    }
  }

  public static void putString(String key, String value) {
    Logger.recordOutput(key, value);
    if (isHighVerbosity()) {
      SmartDashboard.putString(key, value);
    }
  }
}
