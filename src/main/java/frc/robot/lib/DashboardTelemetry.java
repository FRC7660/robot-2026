package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public final class DashboardTelemetry {
  private DashboardTelemetry() {}

  private static final Map<String, StructPublisher<?>> structPublishers = new HashMap<>();

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

  public static void recordOutput(String key, boolean value) {
    Logger.recordOutput(key, value);
  }

  public static void recordOutput(String key, double value) {
    Logger.recordOutput(key, value);
  }

  public static void recordOutput(String key, String value) {
    Logger.recordOutput(key, value);
  }

  public static <T> void recordOutput(String key, Struct<T> struct, T value) {
    Logger.recordOutput(key, struct, value);
  }

  public static <T> void putStruct(String key, Struct<T> struct, T value) {
    if (value == null) {
      return;
    }
    Logger.recordOutput(key, struct, value);
    if (!isHighVerbosity()) {
      return;
    }
    getStructPublisher(key, struct).set(value);
  }

  @SuppressWarnings("unchecked")
  private static <T> StructPublisher<T> getStructPublisher(String key, Struct<T> struct) {
    return (StructPublisher<T>)
        structPublishers.computeIfAbsent(
            key,
            unused ->
                NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getStructTopic(key, struct)
                    .publish());
  }
}
