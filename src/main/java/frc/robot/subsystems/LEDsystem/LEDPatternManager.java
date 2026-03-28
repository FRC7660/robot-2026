package frc.robot.subsystems.LEDsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.DashboardTelemetry;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.Cameras;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.function.Supplier;

public class LEDPatternManager extends SubsystemBase {
  public static final Distance kLedSpacing = Meters.of(1 / 120.0);

  public static final class PatternBank {
    // Basic solid colors
    public final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    public final LEDPattern red = LEDPattern.solid(Color.kRed);
    public final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    public final LEDPattern green = LEDPattern.solid(Color.kGreen);
    public final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public final LEDPattern purple = LEDPattern.solid(Color.kPurple);
    public final LEDPattern orange = LEDPattern.solid(Color.kOrange);
    public final LEDPattern black = LEDPattern.solid(Color.kBlack);
    public final LEDPattern white = LEDPattern.solid(Color.kWhite);

    // Special solid colors
    public final LEDPattern cyan = LEDPattern.solid(Color.kCyan);
    public final LEDPattern magenta = LEDPattern.solid(Color.kMagenta);
    public final LEDPattern lime = LEDPattern.solid(Color.kLime);
    public final LEDPattern pink = LEDPattern.solid(Color.kPink);
    public final LEDPattern teal = LEDPattern.solid(Color.kTeal);
    public final LEDPattern crimson = LEDPattern.solid(Color.kCrimson);
    public final LEDPattern navy = LEDPattern.solid(Color.kNavy);
    public final LEDPattern indigo = LEDPattern.solid(Color.kIndigo);

    /** Staggered colors */
    private final LEDPattern staggerBasic(Color inputColor) {
      LEDPattern finalColor = LEDPattern.solid(Color.kCrimson); // Error color: Crimson
      // This should alternate between the input color and black color every 10% of the total LED
      // length
      // If crimson is seen, most likely LEDPattern.steps is not returning a proper value somehow
      finalColor =
          LEDPattern.steps(
              Map.of(
                  0,
                  inputColor,
                  0.1,
                  Color.kBlack,
                  0.2,
                  inputColor,
                  0.3,
                  Color.kBlack,
                  0.4,
                  inputColor,
                  0.5,
                  Color.kBlack,
                  0.6,
                  inputColor,
                  0.7,
                  Color.kBlack,
                  0.8,
                  inputColor,
                  0.9,
                  Color.kBlack));

      // System.out.println(finalColor);
      return finalColor;
    }

    public final LEDPattern staggerRed = staggerBasic(Color.kRed);
    public final LEDPattern staggerYellow = staggerBasic(Color.kYellow);
    public final LEDPattern staggerGreen = staggerBasic(Color.kGreen);
    public final LEDPattern staggerBlue = staggerBasic(Color.kBlue);
    public final LEDPattern staggerPurple = staggerBasic(Color.kPurple);
    public final LEDPattern staggerOrange = staggerBasic(Color.kOrange);
    public final LEDPattern staggerWhite = staggerBasic(Color.kWhite);

    /** Flickering colors (Supplier type to allow for dynamic updates in real time) */
    private Supplier<LEDPattern> flicker(LEDPattern pattern, Time period) {
      return () -> {
        if (Timer.getTimestamp() % period.in(Seconds) < period.in(Seconds) / 2) {
          return pattern;
        } else {
          return pattern.reversed();
        }
      };
    }

    public final Supplier<LEDPattern> flickerRed = flicker(staggerRed, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerYellow = flicker(staggerYellow, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerGreen = flicker(staggerGreen, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerBlue = flicker(staggerBlue, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerPurple = flicker(staggerPurple, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerOrange = flicker(staggerOrange, Seconds.of(0.5));
    public final Supplier<LEDPattern> flickerWhite = flicker(staggerWhite, Seconds.of(0.5));

    // Focus colors (Partial overlay pattern to indicate subsystem focus)
    public final LEDPattern whiteFocus =
        LEDPattern.steps(Map.of(0.45, Color.kWhite, 0.5, Color.kBlack));
    public final LEDPattern yellowFocus =
        LEDPattern.steps(Map.of(0.45, Color.kYellow, 0.5, Color.kBlack));
    public final LEDPattern greenFocus =
        LEDPattern.steps(Map.of(0.45, Color.kGreen, 0.5, Color.kBlack));
    public final LEDPattern blueFocus =
        LEDPattern.steps(Map.of(0.45, Color.kBlue, 0.5, Color.kBlack));
    public final LEDPattern purpleFocus =
        LEDPattern.steps(Map.of(0.45, Color.kPurple, 0.5, Color.kBlack));

    // Lights off
    public final LEDPattern off = LEDPattern.kOff;
  }

  public static class LightRoutine {
    public LEDPattern activePattern;
    public static focus currentFocus;
    public static LEDPatternManager.PatternBank pBank;

    // Subsystems
    Launch launch;
    Intake intake;
    SwerveSubsystem drivebase;
    Turret turret;
    CommandXboxController driveXbox;

    // Focuses for different subsystems, used to determine which set of routines should be used.
    public enum focus {
      DEFAULT,
      INTAKE,
      SHOOTER,
      DRIVE,
      VISION
    }

    // Priority levels shared across focuses, used to determine which routine within a set should be
    // displayed if multiple conditions are met.
    public enum priorityLevel {
      TEST,
      NORMAL_OPERATION,
      SPECIAL_OPERATION_1,
      SPECIAL_OPERATION_2,
      SPECIAL_OPERATION_3,
      SPECIAL_OPERATION_4,
      SPECIAL_OPERATION_5,
      WARNING,
      ERROR,
      CRITICAL_ERROR
    }

    /**
     * Values for each level to calculate their priority (higher value means higher priority). This
     * is used to override normal patterns with more important ones across all subsystems.
     */
    public HashMap<priorityLevel, Double> priorityValues =
        new HashMap<priorityLevel, Double>() {
          {
            put(priorityLevel.TEST, 0.0);
            put(priorityLevel.NORMAL_OPERATION, 1.0);
            put(priorityLevel.SPECIAL_OPERATION_1, 2.0);
            put(priorityLevel.SPECIAL_OPERATION_2, 2.1);
            put(priorityLevel.SPECIAL_OPERATION_3, 2.2);
            put(priorityLevel.SPECIAL_OPERATION_4, 2.3);
            put(priorityLevel.SPECIAL_OPERATION_5, 2.4);
            put(priorityLevel.WARNING, 3.0);
            put(priorityLevel.ERROR, 4.0);
            put(priorityLevel.CRITICAL_ERROR, 5.0);
          }
        };

    public static class PrioritizedPair {
      public priorityLevel level;
      public Supplier<LEDPattern> routine;

      public PrioritizedPair(priorityLevel level, Supplier<LEDPattern> routine) {
        this.level = level;
        this.routine = routine;
      }

      public LEDPattern getPattern() {
        return routine.get();
      }
    }

    Hashtable<focus, Supplier<PrioritizedPair>> focusRoutines = new Hashtable<>();

    /** Initialization */
    public LightRoutine(
        Launch launchSystem,
        Intake intakeSystem,
        SwerveSubsystem swerveSystem,
        Turret turretSystem,
        CommandXboxController driverXbox) {
      activePattern = LEDPattern.kOff;
      currentFocus = focus.DEFAULT;
      pBank = new PatternBank();

      // Subsystem assignment for easier access in routines
      launch = launchSystem;
      intake = intakeSystem;
      drivebase = swerveSystem;
      turret = turretSystem;
      driveXbox = driverXbox;

      focusRoutines.put(focus.DEFAULT, loadRoutine(focus.DEFAULT));
      focusRoutines.put(focus.INTAKE, loadRoutine(focus.INTAKE));
      focusRoutines.put(focus.SHOOTER, loadRoutine(focus.SHOOTER));
      focusRoutines.put(focus.DRIVE, loadRoutine(focus.DRIVE));
      focusRoutines.put(focus.VISION, loadRoutine(focus.VISION));
    }

    // This method contains all the routines for each focus, and returns the appropriate pattern
    // based on
    // the routine name and current subsystem conditions.
    // Routines are loaded as Suppliers to allow for dynamic updates based on changing conditions.
    public Supplier<PrioritizedPair> loadRoutine(focus focusName) {
      Supplier<PrioritizedPair> routineSupplier;

      switch (focusName) {
        case DEFAULT:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;

                level = priorityLevel.TEST;
                returnPattern =
                    () -> {
                      return pBank.rainbow.scrollAtAbsoluteSpeed(
                          MetersPerSecond.of(0.5), kLedSpacing);
                    };

                LEDPattern focusPattern = pBank.whiteFocus.overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return focusPattern;
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;

        case INTAKE:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;
                double cacheRollerOutput = intake.getRollerOutput();

                if (cacheRollerOutput > 0.1) {
                  DashboardTelemetry.putString(
                      "LEDS/" + focusName.toString(),
                      "ACTIVE DISPLAY - ROLLER OUTPUT: " + cacheRollerOutput * 100 + "%");
                  level = priorityLevel.SPECIAL_OPERATION_1;
                  returnPattern =
                      () -> {
                        // Flicker green and overlay with red at a brightness corresponding to the
                        // roller speed.
                        return pBank
                            .flickerGreen
                            .get()
                            .overlayOn(pBank.lime)
                            .blend(pBank.red.atBrightness(Percent.of(cacheRollerOutput)));
                      };
                } else {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank.green;
                      };
                }

                LEDPattern focusPattern = pBank.greenFocus.overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return focusPattern;
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;

        case SHOOTER:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;
                double cacheVelocity = launch.getVelocity().in(RPM);
                double cacheOptimal = launch.getOptimalVelocity(turret).in(RPM);

                if (cacheVelocity > 50 || turret.autoSetAngle().isScheduled()) {
                  DashboardTelemetry.putString(
                      "LEDS/" + focusName.toString(),
                      "ACTIVE DISPLAY - VELOCITY: " + cacheVelocity + " RPM");
                  level = priorityLevel.SPECIAL_OPERATION_3;
                  returnPattern =
                      () -> {
                        // 0 -> 100 correlating with difference from optimal velocity
                        double launchPercent =
                            (100 - (Math.abs(cacheOptimal - cacheVelocity) / cacheOptimal) * 100)
                                * 0.9;
                        // 0 approaching 100 based on absolute current velocity
                        double absolutePercent = 100 - ((5000 - cacheVelocity) / 5000) * 100;
                        return pBank
                            .red
                            .atBrightness(Percent.of(100 - launchPercent))
                            .blend(
                                LEDPattern.gradient(
                                        LEDPattern.GradientType.kDiscontinuous,
                                        Color.kLime,
                                        Color.kLime,
                                        Color.kLime,
                                        Color.kGold,
                                        Color.kRed)
                                    .atBrightness(Percent.of(launchPercent)))
                            .mask(
                                LEDPattern.steps(
                                    Map.of(
                                        0,
                                        Color.kWhite,
                                        (launchPercent / 100) * (0.45 / 2),
                                        Color.kBlack,
                                        (0.45 / 2),
                                        Color.kBlack,
                                        0.45 - (launchPercent / 100) * (0.45 / 2),
                                        Color.kWhite,
                                        0.5,
                                        Color.kWhite,
                                        0.5 + (absolutePercent / 100) / 2,
                                        Color.kBlack)));
                      };
                } else {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank.yellow;
                      };
                }

                Double angleFactor = (1 - (175 - turret.getCurrentAngleDegrees()) / 350) / 2;
                LEDPattern aimPattern =
                    LEDPattern.steps(
                            Map.of(
                                0,
                                Color.kBlack,
                                0.5,
                                Color.kBlack,
                                0.5 + angleFactor - 0.02,
                                Color.kMagenta,
                                0.5 + angleFactor + 0.02,
                                Color.kBlack))
                        .breathe(Seconds.of(0.5))
                        .overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return aimPattern;
                    };

                LEDPattern focusPattern = pBank.yellowFocus.overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return focusPattern;
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;

        case DRIVE:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;

                if (drivebase.navxConnected() == false) {
                  DashboardTelemetry.putString(
                      "LEDS/" + focusName.toString(), "NAVX ERROR - CHECK CONNECTION");
                  level = priorityLevel.CRITICAL_ERROR;
                  // If the gyro is not connected, breathe red to indicate an error.
                  // This is a critical error since the robot relies on the gyro for
                  // field-oriented control, so it is prioritized over other potential errors.
                  returnPattern =
                      () -> {
                        return pBank.red.breathe(Seconds.of(1));
                      };
                } else {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank.blue;
                      };
                }

                LEDPattern focusPattern = pBank.blueFocus.overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return focusPattern;
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;

        case VISION:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;
                LEDPattern sightedPattern;

                // Default level and return pattern if no targets are visible is solid purple.
                level = priorityLevel.NORMAL_OPERATION;
                returnPattern =
                    () -> {
                      return pBank.purple;
                    };

                boolean camDisconnect = false;
                int sightings = 0;
                String[] sightedCameras = new String[4];
                for (Cameras cameraKey : Cameras.values()) {
                  if (!cameraKey.getCamera().isConnected() && cameraKey != Cameras.FRONT_RIGHT) {
                    camDisconnect = true;
                    DashboardTelemetry.putString(
                        "LEDS/" + focusName.toString(),
                        "CAMERA ERROR - MISSING " + cameraKey.toString());
                  } else if (SmartDashboard.getBoolean(
                      "Vision/" + cameraKey.toString() + "/TagVisible", false)) {
                    sightings++;
                    sightedCameras[sightings - 1] = cameraKey.toString();
                  }
                }
                // If any camera has a visible target, overlay flashing pink to indicate vision
                // tracking.
                // Faster blinking = more targets visible.
                // Apply sighting indicator and determine priority level.
                // Only activates if R3 is down.
                if (driveXbox.rightStick().getAsBoolean() == true && camDisconnect == false) {
                  level = priorityLevel.SPECIAL_OPERATION_2;
                  sightedPattern =
                      pBank
                          .pink
                          .blink(Seconds.of(1 / (0.1 + sightings)))
                          .overlayOn(returnPattern.get());
                  DashboardTelemetry.putString(
                      "LEDS/" + focusName.toString(),
                      "ACTIVE DISPLAY - " + sightings + " TARGET(s) VISIBLE");
                } else if (camDisconnect == true) {
                  level = priorityLevel.ERROR;
                  sightedPattern = pBank.staggerPurple.overlayOn(pBank.flickerRed.get());
                } else {
                  sightedPattern = returnPattern.get();
                }

                returnPattern =
                    () -> {
                      return sightedPattern;
                    };

                LEDPattern focusPattern = pBank.purpleFocus.overlayOn(returnPattern.get());
                returnPattern =
                    () -> {
                      return focusPattern.atBrightness(Percent.of(25));
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;

        default:
          routineSupplier =
              () -> {
                priorityLevel level;
                Supplier<LEDPattern> returnPattern;
                level = priorityLevel.TEST;
                returnPattern =
                    () -> {
                      return pBank.white;
                    };
                return new PrioritizedPair(level, returnPattern);
              };
          break;
      }

      DashboardTelemetry.putString("LEDS/" + focusName.toString(), "routines loaded");
      return routineSupplier;
    }

    public LEDPattern update() {
      PrioritizedPair currentRoutinePair = focusRoutines.get(currentFocus).get();
      Boolean allNormal =
          true; // Used to check if all routines are at normal operation, changing how they are
      // displayed.
      for (focus focus : focusRoutines.keySet()) {
        PrioritizedPair routinePair = focusRoutines.get(focus).get();
        // Compares the priority level of the current routine with the one being iterated through,
        // updating the focus if the new one has a higher priority (higher value in the
        // priorityValues map).
        if (priorityValues.get(routinePair.level) > priorityValues.get(currentRoutinePair.level)) {
          currentFocus = focus;
        }
        if (priorityValues.get(routinePair.level)
            <= priorityValues.get(priorityLevel.NORMAL_OPERATION)) {
          DashboardTelemetry.putString(
              "LEDS/" + focus.toString(), "INACTIVE DISPLAY - NORMAL OPERATION");
        } else {
          allNormal = false;
        }
      }
      ;

      // Publish the displayed focus' status if it is not already published by the routine (ex.
      // shooter publishing with velocity)
      if (!SmartDashboard.getString("LEDS/" + currentFocus.toString(), "INACTIVE")
          .contains("ACTIVE DISPLAY")) {
        DashboardTelemetry.putString(
            "LEDS/" + currentFocus.toString(),
            "ACTIVE DISPLAY - " + focusRoutines.get(currentFocus).get().level.toString());
      }

      if (allNormal) {
        // Show the default focus if all routines are at normal operation
        currentFocus = focus.DEFAULT;
      }

      activePattern = focusRoutines.get(currentFocus).get().getPattern();
      return activePattern;
    }
  }
}
