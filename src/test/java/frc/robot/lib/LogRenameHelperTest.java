package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.time.Instant;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class LogRenameHelperTest {

  @TempDir Path tempDir;

  @Test
  public void planRenamesQualificationPicksLatestPerSource() throws IOException {
    Path logsDir = tempDir.resolve("logs");
    Files.createDirectories(logsDir);
    Files.createDirectories(logsDir.resolve("MISAL_Q71"));
    Files.createDirectories(logsDir.resolve("renamed"));

    // Matching files.
    Path akit1 = touch(logsDir.resolve("akit_26-03-29_15-28-09_misal_q71.wpilog"), 1000);
    Path akit2 = touch(logsDir.resolve("akit_26-03-29_15-28-10_misal_q71.wpilog"), 2000);
    Path frc = touch(logsDir.resolve("FRC_20260329_152813_MISAL_Q71.wpilog"), 1500);
    Path phoenix =
        touch(logsDir.resolve("MISAL_Q71").resolve("MISAL_Q71_rio_2026-03-29_15-28-03.hoot"), 1200);
    Path rev = touch(logsDir.resolve("REV_20260329_144449.revlog"), 1800);

    // Non-matching or should-be-ignored.
    touch(logsDir.resolve("akit_26-03-29_15-28-11_misal_q72.wpilog"), 3000);
    touch(logsDir.resolve("renamed").resolve("2026-misal-q71-akit.wpilog"), 9999);

    var info =
        new LogRenameHelper.FmsInfo(false, "MISAL", DriverStation.MatchType.Qualification, 71, 0);
    List<LogRenameHelper.RenameAction> actions = LogRenameHelper.planRenames(logsDir, info);

    Map<String, Path> destToSrc = toDestMap(actions);
    assertEquals(4, destToSrc.size());

    assertEquals(akit2, destToSrc.get("2026-misal-q71-akit.wpilog"));
    assertEquals(frc, destToSrc.get("2026-misal-q71-frc.wpilog"));
    assertEquals(phoenix, destToSrc.get("2026-misal-q71-phoenix.hoot"));
    assertEquals(rev, destToSrc.get("2026-misal-q71-rev.revlog"));
  }

  @Test
  public void planRenamesPracticeOnlyRenamesRev() throws IOException {
    Path logsDir = tempDir.resolve("logs-practice");
    Files.createDirectories(logsDir);

    Path rev = touch(logsDir.resolve("REV_20260329_144449.revlog"), 1000);
    touch(logsDir.resolve("FRC_20260329_152813_MISAL_Q71.wpilog"), 2000);

    var info = new LogRenameHelper.FmsInfo(true, "MISAL", DriverStation.MatchType.Practice, 71, 0);
    List<LogRenameHelper.RenameAction> actions = LogRenameHelper.planRenames(logsDir, info);

    Map<String, Path> destToSrc = toDestMap(actions);
    assertEquals(1, destToSrc.size());
    assertEquals(rev, destToSrc.get("2026-misal-p71-rev.revlog"));
  }

  @Test
  public void planRenamesReplayAddsInstanceSuffix() throws IOException {
    Path logsDir = tempDir.resolve("logs-replay");
    Files.createDirectories(logsDir);

    Path rev = touch(logsDir.resolve("REV_20260329_144449.revlog"), 1000);

    var info =
        new LogRenameHelper.FmsInfo(true, "MISAL", DriverStation.MatchType.Qualification, 35, 2);
    List<LogRenameHelper.RenameAction> actions = LogRenameHelper.planRenames(logsDir, info);

    Map<String, Path> destToSrc = toDestMap(actions);
    assertEquals(1, destToSrc.size());
    assertEquals(rev, destToSrc.get("2026-misal-q35r2-rev.revlog"));
  }

  private static Path touch(Path path, long millis) throws IOException {
    Files.createDirectories(path.getParent());
    if (!Files.exists(path)) {
      Files.createFile(path);
    }
    Files.setLastModifiedTime(path, FileTime.from(Instant.ofEpochMilli(millis)));
    return path;
  }

  private static Map<String, Path> toDestMap(List<LogRenameHelper.RenameAction> actions) {
    Map<String, Path> map = new HashMap<>();
    for (LogRenameHelper.RenameAction action : actions) {
      String name = action.destination().getFileName().toString();
      assertTrue(map.put(name, action.source()) == null);
    }
    return map;
  }
}
