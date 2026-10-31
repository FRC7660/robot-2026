package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Build-time validation that every {@code NamedCommand} referenced inside a PathPlanner {@code
 * .auto} routine is registered in Java code.
 *
 * <p>In 2025, {@code AutoBuilder.buildAutoChooser()} populated the auto chooser from the {@code
 * .auto} files on disk, so auto-name-to-file always matched. The actual string-mismatch bug was the
 * other direction: a {@code .auto} file references a {@code NamedCommand} by name (e.g. {@code
 * "Shoot Coral"}, {@code "Arm Out"}) and PathPlanner resolves it at runtime via {@code
 * NamedCommands}. If that string doesn't exactly match a {@code NamedCommands.registerCommand(...)}
 * call in {@link frc.robot.RobotContainer}, the routine silently does nothing at runtime.
 *
 * <p>This same failure mode still exists in 2026: {@code .auto} files reference {@code
 * NamedCommand} names that must match Java registrations. This test derives both sides at build
 * time -- it parses the {@code .auto} files for referenced names and parses {@code RobotContainer}
 * for registered names -- so nothing is duplicated or hardcoded in the test itself. The build fails
 * when any referenced name lacks a registration, catching the mismatch before it reaches the robot.
 */
class AutonomousRoutineNamesTest {

  private static final Path AUTOS_DIR = Paths.get("src/main/deploy/pathplanner/autos");
  private static final Path ROBOT_CONTAINER =
      Paths.get("src/main/java/frc/robot/RobotContainer.java");

  @Test
  void everyNamedCommandReferencedByAnAutoIsRegistered() throws IOException {
    Set<String> registeredNames = extractRegisteredNamedCommandNames(ROBOT_CONTAINER);
    Set<String> referencedNames = extractReferencedNamedCommandNames(AUTOS_DIR);

    assertFalse(
        registeredNames.isEmpty(),
        "No NamedCommands found registered in "
            + ROBOT_CONTAINER
            + "; nothing to validate against.");

    assertFalse(
        referencedNames.isEmpty(),
        "No NamedCommands found referenced in .auto files under "
            + AUTOS_DIR
            + "; nothing to validate.");

    List<String> unregistered =
        referencedNames.stream().filter(name -> !registeredNames.contains(name)).sorted().toList();

    assertTrue(
        unregistered.isEmpty(),
        "The following NamedCommand names are referenced in .auto files under "
            + AUTOS_DIR
            + " but are not registered via NamedCommands.registerCommand in "
            + ROBOT_CONTAINER
            + ": "
            + unregistered
            + ". Add the missing registration or correct the name in the .auto file.");
  }

  /**
   * Parses {@code NamedCommands.registerCommand("NAME", ...)} calls from source so the list of
   * registered names is generated at build time rather than duplicated in the test.
   */
  private static Set<String> extractRegisteredNamedCommandNames(Path source) throws IOException {
    Set<String> names = new LinkedHashSet<>();
    String content = Files.readString(source);
    // Matches:  NamedCommands.registerCommand("NAME", ...)
    //           (name string may be on the line after registerCommand()
    Pattern pattern = Pattern.compile("NamedCommands\\.registerCommand\\s*\\(\\s*\"([^\"]+)\"");
    Matcher matcher = pattern.matcher(content);
    while (matcher.find()) {
      names.add(matcher.group(1));
    }
    return names;
  }

  /**
   * Parses all {@code .auto} files under {@code autosDir} and extracts the {@code name} value of
   * every {@code "type": "named"} command, so the list of referenced names is generated at build
   * time from the actual routine files.
   */
  private static Set<String> extractReferencedNamedCommandNames(Path autosDir) throws IOException {
    Set<String> names = new LinkedHashSet<>();
    try (var files = Files.list(autosDir)) {
      List<Path> autoFiles = files.filter(p -> p.toString().endsWith(".auto")).sorted().toList();
      for (Path autoFile : autoFiles) {
        String content = Files.readString(autoFile);
        // Matches:  "type": "named", ... "name": "Some Name"
        // (tolerant of key ordering within the named command object)
        Pattern pattern =
            Pattern.compile("\"type\"\\s*:\\s*\"named\"[\\s\\S]*?\"name\"\\s*:\\s*\"([^\"]+)\"");
        Matcher matcher = pattern.matcher(content);
        while (matcher.find()) {
          names.add(matcher.group(1));
        }
      }
    }
    return names;
  }
}
