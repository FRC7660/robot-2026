package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * High-performance buffered logger that writes timestamped log files to local disk.
 *
 * <p>A new log file is created each time the robot program starts. Messages are buffered in memory
 * and flushed to disk either when the buffer reaches a line-count threshold or on a timed schedule
 * via {@link #periodic()}. Output is also mirrored to {@code System.out} so it remains visible in
 * the Driver Station console.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // In robotInit():
 * BufferedLogger.getInstance();
 *
 * // In robotPeriodic():
 * BufferedLogger.getInstance().periodic();
 *
 * // Anywhere you previously used System.out.println / printf:
 * BufferedLogger.getInstance().println("[MyTag] something happened");
 * BufferedLogger.getInstance().printf("[MyTag] value=%d%n", 42);
 * }</pre>
 */
public final class BufferedLogger {

  private static BufferedLogger instance;

  private static final String LOG_DIR = "/home/lvuser/logs";
  private static final int BUFFER_CAPACITY = 8192;
  private static final int FLUSH_LINE_THRESHOLD = 50;
  private static final double FLUSH_PERIOD_SEC = 0.5;

  private final StringBuilder buffer = new StringBuilder(BUFFER_CAPACITY);
  private final BufferedWriter writer;
  private final String logFilePath;
  private int lineCount;
  private double lastFlushSec;

  private BufferedLogger() {
    String timestamp =
        LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
    logFilePath = LOG_DIR + "/robot_" + timestamp + ".log";

    BufferedWriter w = null;
    try {
      new File(LOG_DIR).mkdirs();
      w = new BufferedWriter(new FileWriter(logFilePath), BUFFER_CAPACITY);
    } catch (IOException e) {
      System.err.println("[BufferedLogger] Failed to open log file: " + e.getMessage());
    }
    writer = w;
    System.out.println("========================================");
    System.out.println("[BufferedLogger] Log file: " + logFilePath);
    System.out.println("========================================");
  }

  /** Get the singleton instance, creating it (and the log file) on first call. */
  public static synchronized BufferedLogger getInstance() {
    if (instance == null) {
      instance = new BufferedLogger();
    }
    return instance;
  }

  /**
   * Log a single line. A FPGA-timestamp prefix is prepended automatically.
   *
   * @param message the message to log
   */
  public void println(String message) {
    String line = String.format("[%.3f] %s", Timer.getFPGATimestamp(), message);
    buffer.append(line).append('\n');
    lineCount++;
    System.out.println(line);
    if (lineCount >= FLUSH_LINE_THRESHOLD) {
      flush();
    }
  }

  /**
   * Log a formatted message. A FPGA-timestamp prefix is prepended automatically. The format string
   * should NOT include a trailing newline — one is added automatically.
   *
   * @param format format string (as in {@link String#format})
   * @param args format arguments
   */
  public void printf(String format, Object... args) {
    String formatted = String.format(format, args);
    // Strip trailing newline if caller included one; we add our own
    if (formatted.endsWith("\n")) {
      formatted = formatted.substring(0, formatted.length() - 1);
    }
    String line = String.format("[%.3f] %s", Timer.getFPGATimestamp(), formatted);
    buffer.append(line).append('\n');
    lineCount++;
    System.out.println(line);
    if (lineCount >= FLUSH_LINE_THRESHOLD) {
      flush();
    }
  }

  /**
   * Call from {@code robotPeriodic()} to flush buffered messages on a timed schedule. This ensures
   * messages reach disk even during low-activity periods without blocking every loop iteration.
   */
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (lineCount > 0 && now - lastFlushSec >= FLUSH_PERIOD_SEC) {
      flush();
      lastFlushSec = now;
    }
  }

  /** Flush all buffered messages to disk immediately. */
  public void flush() {
    if (writer == null || lineCount == 0) {
      return;
    }
    try {
      writer.write(buffer.toString());
      writer.flush();
    } catch (IOException e) {
      System.err.println("[BufferedLogger] Flush failed: " + e.getMessage());
    }
    buffer.setLength(0);
    lineCount = 0;
  }

  /** Flush and close the underlying file writer. Call on robot shutdown if possible. */
  public void close() {
    flush();
    try {
      if (writer != null) {
        writer.close();
      }
    } catch (IOException e) {
      System.err.println("[BufferedLogger] Close failed: " + e.getMessage());
    }
  }

  /** @return the absolute path of the current log file */
  public String getLogFilePath() {
    return logFilePath;
  }
}
