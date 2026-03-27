// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Base64;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

public final class WPILogToText {
  private static final class EntryInfo {
    final String name;
    final String type;

    EntryInfo(String name, String type) {
      this.name = name;
      this.type = type;
    }
  }

  private WPILogToText() {}

  public static void main(String[] args) throws IOException {
    boolean lenient = false;
    java.util.List<String> positional = new java.util.ArrayList<>();
    for (String arg : args) {
      if ("--lenient".equals(arg)) {
        lenient = true;
      } else {
        positional.add(arg);
      }
    }

    if (positional.size() < 1 || positional.size() > 2) {
      System.err.println("Usage: WPILogToText [--lenient] <input.wpilog> [output.txt]");
      System.exit(2);
    }

    Path inputPath = Paths.get(positional.get(0));
    DataLogReader reader = new DataLogReader(inputPath.toString());
    if (!reader.isValid()) {
      System.err.println("Invalid or unreadable log: " + inputPath);
      System.exit(1);
    }

    Writer out;
    if (positional.size() == 2) {
      out = Files.newBufferedWriter(Paths.get(positional.get(1)), StandardCharsets.UTF_8);
    } else {
      out = new BufferedWriter(new OutputStreamWriter(System.out, StandardCharsets.UTF_8));
    }

    Map<Integer, EntryInfo> entries = new HashMap<>();
    Long firstTimestamp = null;

    try {
      java.util.Iterator<DataLogRecord> iterator = reader.iterator();
      while (true) {
        DataLogRecord record;
        try {
          if (!iterator.hasNext()) {
            break;
          }
          record = iterator.next();
        } catch (IllegalArgumentException e) {
          if (lenient) {
            System.err.println(
                "Warning: encountered corrupt or truncated record; output may be incomplete.");
            break;
          }
          throw e;
        }
        if (record.isStart()) {
          StartRecordData data = record.getStartData();
          entries.put(data.entry, new EntryInfo(data.name, data.type));
          continue;
        }
        if (record.isFinish()) {
          entries.remove(record.getFinishEntry());
          continue;
        }
        if (record.isSetMetadata()) {
          // Metadata doesn't affect data decoding.
          continue;
        }

        long timestamp = record.getTimestamp();
        if (firstTimestamp == null) {
          firstTimestamp = timestamp;
        }
        long monotonic = timestamp - firstTimestamp;
        double monotonicSeconds = monotonic / 1e6;
        String ts = String.format(Locale.ROOT, "%8.3f", monotonicSeconds);

        EntryInfo info = entries.get(record.getEntry());
        String name = info != null ? info.name : "entry/" + record.getEntry();
        String type = info != null ? info.type : "raw";
        String value = decodeValue(type, record);

        out.write(ts);
        out.write('\t');
        out.write(name);
        out.write('\t');
        out.write(value);
        out.write('\n');
      }
    } finally {
      out.flush();
      if (args.length == 2) {
        out.close();
      }
    }
  }

  private static String decodeValue(String type, DataLogRecord record) {
    switch (type) {
      case "boolean":
        return record.getBoolean() ? "true" : "false";
      case "int64":
        return Long.toString(record.getInteger());
      case "float":
        return Float.toString(record.getFloat());
      case "double":
        return Double.toString(record.getDouble());
      case "string":
        return quoteString(record.getString());
      case "boolean[]":
        return booleanArrayToString(record.getBooleanArray());
      case "int64[]":
        return longArrayToString(record.getIntegerArray());
      case "float[]":
        return floatArrayToString(record.getFloatArray());
      case "double[]":
        return doubleArrayToString(record.getDoubleArray());
      case "string[]":
        return stringArrayToString(record.getStringArray());
      default:
        byte[] raw = record.getRaw();
        if (raw.length == 0) {
          return "";
        }
        return "base64:" + Base64.getEncoder().encodeToString(raw);
    }
  }

  private static String booleanArrayToString(boolean[] data) {
    StringBuilder out = new StringBuilder();
    out.append('[');
    for (int i = 0; i < data.length; i++) {
      if (i != 0) {
        out.append(',');
      }
      out.append(data[i] ? "true" : "false");
    }
    out.append(']');
    return out.toString();
  }

  private static String longArrayToString(long[] data) {
    StringBuilder out = new StringBuilder();
    out.append('[');
    for (int i = 0; i < data.length; i++) {
      if (i != 0) {
        out.append(',');
      }
      out.append(data[i]);
    }
    out.append(']');
    return out.toString();
  }

  private static String floatArrayToString(float[] data) {
    StringBuilder out = new StringBuilder();
    out.append('[');
    for (int i = 0; i < data.length; i++) {
      if (i != 0) {
        out.append(',');
      }
      out.append(data[i]);
    }
    out.append(']');
    return out.toString();
  }

  private static String doubleArrayToString(double[] data) {
    StringBuilder out = new StringBuilder();
    out.append('[');
    for (int i = 0; i < data.length; i++) {
      if (i != 0) {
        out.append(',');
      }
      out.append(data[i]);
    }
    out.append(']');
    return out.toString();
  }

  private static String stringArrayToString(String[] data) {
    StringBuilder out = new StringBuilder();
    out.append('[');
    for (int i = 0; i < data.length; i++) {
      if (i != 0) {
        out.append(',');
      }
      out.append(quoteString(data[i]));
    }
    out.append(']');
    return out.toString();
  }

  private static String quoteString(String value) {
    StringBuilder out = new StringBuilder(value.length() + 2);
    out.append('"');
    for (int i = 0; i < value.length(); i++) {
      char c = value.charAt(i);
      switch (c) {
        case '\\':
          out.append("\\\\");
          break;
        case '"':
          out.append("\\\"");
          break;
        case '\n':
          out.append("\\n");
          break;
        case '\r':
          out.append("\\r");
          break;
        case '\t':
          out.append("\\t");
          break;
        default:
          if (c < 0x20) {
            out.append(String.format("\\u%04x", (int) c));
          } else {
            out.append(c);
          }
      }
    }
    out.append('"');
    return out.toString();
  }
}
