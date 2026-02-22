package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * High-performance buffered logger that writes timestamped log files to local disk.
 *
 * <p>A new log file is created each time the robot program starts. Messages are queued lock-free
 * from the main robot thread and written to disk by a dedicated background thread, so disk I/O
 * never blocks the 50Hz control loop. Output is also mirrored to {@code System.out} so it remains
 * visible in the Driver Station console.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // In robotInit():
 * BufferedLogger.getInstance();
 *
 * // Anywhere you previously used System.out.println / printf:
 * BufferedLogger.getInstance().println("[MyTag] something happened");
 * BufferedLogger.getInstance().printf("[MyTag] value=%d", 42);
 * }</pre>
 */
public final class BufferedLogger {

  private static BufferedLogger instance;

  private static final String LOG_DIR = "/home/lvuser/logs";
  private static final int BUFFER_CAPACITY = 8192;
  private static final long FLUSH_INTERVAL_MS = 500;

  private final ConcurrentLinkedQueue<String> queue = new ConcurrentLinkedQueue<>();
  private final BufferedWriter writer;
  private final String logFilePath;
  private final Thread writerThread;
  private volatile boolean running = true;

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

    writerThread = new Thread(this::writerLoop, "BufferedLogger");
    writerThread.setDaemon(true);
    writerThread.start();
  }

  /** Get the singleton instance, creating it (and the log file) on first call. */
  public static synchronized BufferedLogger getInstance() {
    if (instance == null) {
      instance = new BufferedLogger();
    }
    return instance;
  }

  /**
   * Log a single line. A FPGA-timestamp prefix is prepended automatically. This method is lock-free
   * and safe to call from the main robot thread.
   *
   * @param message the message to log
   */
  public void println(String message) {
    String line = String.format("[%.3f] %s", Timer.getFPGATimestamp(), message);
    queue.add(line);
    System.out.println(line);
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
    queue.add(line);
    System.out.println(line);
  }

  /** Background loop that drains the queue and writes to disk. */
  private void writerLoop() {
    StringBuilder batch = new StringBuilder(BUFFER_CAPACITY);
    while (running) {
      try {
        Thread.sleep(FLUSH_INTERVAL_MS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        break;
      }
      drainTo(batch);
    }
    // Final drain on shutdown
    drainTo(batch);
  }

  /** Drain all queued messages into the writer. */
  private void drainTo(StringBuilder batch) {
    if (writer == null) {
      queue.clear();
      return;
    }
    batch.setLength(0);
    String line;
    while ((line = queue.poll()) != null) {
      batch.append(line).append('\n');
    }
    if (batch.length() == 0) {
      return;
    }
    try {
      writer.write(batch.toString());
      writer.flush();
    } catch (IOException e) {
      System.err.println("[BufferedLogger] Flush failed: " + e.getMessage());
    }
  }

  /** Flush and close the underlying file writer. Call on robot shutdown if possible. */
  public void close() {
    running = false;
    try {
      writerThread.join(5000);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    try {
      if (writer != null) {
        writer.close();
      }
    } catch (IOException e) {
      System.err.println("[BufferedLogger] Close failed: " + e.getMessage());
    }
  }

  /**
   * @return the absolute path of the current log file
   */
  public String getLogFilePath() {
    return logFilePath;
  }
}
