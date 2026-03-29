package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

public final class LogRenameHelper {
  private LogRenameHelper() {}

  public record FmsInfo(
      boolean isSimulation,
      String eventName,
      DriverStation.MatchType matchType,
      int matchNumber,
      int replayNumber) {}

  public record RenameAction(Path source, Path destination) {}

  private static final Pattern REV_YEAR = Pattern.compile("(?i)^rev_(\\d{4})\\d{4}_\\d{6}");
  private static final Pattern FRC_YEAR = Pattern.compile("(?i)^frc_(\\d{4})\\d{4}_\\d{6}");
  private static final Pattern AKIT_YEAR =
      Pattern.compile("(?i)^akit_(\\d{2})-(\\d{2})-(\\d{2})_\\d{2}-\\d{2}-\\d{2}");
  private static final Pattern RIO_YEAR =
      Pattern.compile("(?i)_rio_(\\d{4})-(\\d{2})-(\\d{2})_\\d{2}-\\d{2}-\\d{2}");
  private static final Pattern MATCH_TOKEN = Pattern.compile(".*[-_]([qe])\\d{1,2}.*");

  public static List<RenameAction> planRenames(Path logsDir, FmsInfo info) {
    if (info == null) {
      return List.of();
    }
    if (info.matchType() == null || info.matchType() == DriverStation.MatchType.None) {
      return List.of();
    }
    if (info.matchNumber() <= 0) {
      return List.of();
    }
    if (logsDir == null || !Files.isDirectory(logsDir)) {
      return List.of();
    }

    String eventCode = sanitizeEventCode(info.eventName());
    if (eventCode.isEmpty()) {
      return List.of();
    }

    String matchTokenBase;
    switch (info.matchType()) {
      case Qualification -> matchTokenBase = String.format("q%d", info.matchNumber());
      case Practice -> matchTokenBase = String.format("p%d", info.matchNumber());
      case Elimination -> matchTokenBase = String.format("e%d", info.matchNumber());
      default -> {
        return List.of();
      }
    }

    String matchTokenWithReplay = matchTokenBase;
    if (info.replayNumber() > 0) {
      matchTokenWithReplay = String.format("%sr%d", matchTokenBase, info.replayNumber());
    }

    boolean allowNonRev =
        info.matchType() == DriverStation.MatchType.Qualification
            || info.matchType() == DriverStation.MatchType.Elimination;

    Path renamedDir = logsDir.resolve("renamed");

    List<LogRule> rules = new ArrayList<>();
    rules.add(new LogRule("rev", ".revlog", LogRule.Kind.REV, true));
    if (allowNonRev) {
      rules.add(new LogRule("akit", ".wpilog", LogRule.Kind.AKIT, false));
      rules.add(new LogRule("phoenix", ".hoot", LogRule.Kind.PHOENIX, false));
      rules.add(new LogRule("frc", ".wpilog", LogRule.Kind.FRC, false));
    }

    List<RenameAction> actions = new ArrayList<>();
    for (LogRule rule : rules) {
      Path latest = null;
      FileTime latestTime = null;
      try (Stream<Path> stream = Files.walk(logsDir)) {
        for (Path path : (Iterable<Path>) stream::iterator) {
          if (!Files.isRegularFile(path)) {
            continue;
          }
          if (path.startsWith(renamedDir)) {
            continue;
          }
          String name = path.getFileName().toString();
          if (!rule.matches(name)) {
            continue;
          }
          if (!rule.allowAnyMatchToken) {
            String lower = name.toLowerCase();
            if (!lower.contains(eventCode)) {
              continue;
            }
            if (!MATCH_TOKEN.matcher(lower).matches()) {
              continue;
            }
            if (!lower.contains(matchTokenBase)) {
              continue;
            }
          }
          FileTime modified = Files.getLastModifiedTime(path);
          if (latestTime == null || modified.compareTo(latestTime) > 0) {
            latestTime = modified;
            latest = path;
          }
        }
      } catch (IOException e) {
        return List.of();
      }

      if (latest == null) {
        continue;
      }

      int year = extractYearFromFilename(latest.getFileName().toString());
      String destName =
          String.format(
              "%04d-%s-%s-%s%s", year, eventCode, matchTokenWithReplay, rule.source, rule.ext);
      actions.add(new RenameAction(latest, renamedDir.resolve(destName)));
    }
    return actions;
  }

  private static String sanitizeEventCode(String eventName) {
    if (eventName == null) {
      return "";
    }
    return eventName.trim().toLowerCase().replaceAll("[^a-z0-9]", "");
  }

  private static int extractYearFromFilename(String filename) {
    Matcher m = REV_YEAR.matcher(filename);
    if (m.find()) {
      return Integer.parseInt(m.group(1));
    }
    m = FRC_YEAR.matcher(filename);
    if (m.find()) {
      return Integer.parseInt(m.group(1));
    }
    m = RIO_YEAR.matcher(filename);
    if (m.find()) {
      return Integer.parseInt(m.group(1));
    }
    m = AKIT_YEAR.matcher(filename);
    if (m.find()) {
      int yy = Integer.parseInt(m.group(1));
      return 2000 + yy;
    }
    return LocalDate.now().getYear();
  }

  private static final class LogRule {
    enum Kind {
      AKIT,
      PHOENIX,
      REV,
      FRC
    }

    final String source;
    final String ext;
    final Kind kind;
    final boolean allowAnyMatchToken;

    LogRule(String source, String ext, Kind kind, boolean allowAnyMatchToken) {
      this.source = source;
      this.ext = ext;
      this.kind = kind;
      this.allowAnyMatchToken = allowAnyMatchToken;
    }

    boolean matches(String name) {
      String lower = name.toLowerCase();
      return switch (kind) {
        case AKIT -> lower.startsWith("akit") && lower.endsWith(".wpilog");
        case PHOENIX -> lower.endsWith(".hoot");
        case REV -> lower.startsWith("rev") && lower.endsWith(".revlog");
        case FRC -> lower.startsWith("frc") && lower.endsWith(".wpilog");
      };
    }
  }
}
