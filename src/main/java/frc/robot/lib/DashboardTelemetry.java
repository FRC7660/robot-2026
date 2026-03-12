package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public final class DashboardTelemetry {
  private DashboardTelemetry() {}

  public static boolean isHighVerbosity() {
    return Constants.Telemetry.isHighVerbosity();
  }

  public static void putBoolean(String key, boolean value) {
    if (isHighVerbosity()) {
      SmartDashboard.putBoolean(key, value);
    }
  }

  public static void putNumber(String key, double value) {
    if (isHighVerbosity()) {
      SmartDashboard.putNumber(key, value);
    }
  }

  public static void putString(String key, String value) {
    if (isHighVerbosity()) {
      SmartDashboard.putString(key, value);
    }
  }
}
