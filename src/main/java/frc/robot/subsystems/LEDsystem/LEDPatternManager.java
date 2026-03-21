package frc.robot.subsystems.LEDsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
        LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.1, Color.kBlack));
    public final LEDPattern yellowFocus =
        LEDPattern.steps(Map.of(0.0, Color.kYellow, 0.1, Color.kBlack));
    public final LEDPattern greenFocus =
        LEDPattern.steps(Map.of(0.0, Color.kGreen, 0.1, Color.kBlack));
    public final LEDPattern blueFocus =
        LEDPattern.steps(Map.of(0.0, Color.kBlue, 0.1, Color.kBlack));

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

    // Focuses for different subsystems, used to determine which set of routines should be used.
    public enum focus {
      DEFAULT,
      INTAKE,
      SHOOTER,
      DRIVE
    }

    // Priority levels shared across focuses, used to determine which routine within a set should be
    // displayed if multiple conditions are met.
    public enum priorityLevel {
      TEST,
      NORMAL_OPERATION,
      SPECIAL_OPERATION,
      WARNING,
      ERROR,
      CRITICAL_ERROR
    }

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
        Turret turretSystem) {
      activePattern = LEDPattern.kOff;
      currentFocus = focus.DEFAULT;
      pBank = new PatternBank();

      // Subsystem assignment for easier access in routines
      launch = launchSystem;
      intake = intakeSystem;
      drivebase = swerveSystem;
      turret = turretSystem;

      focusRoutines.put(focus.DEFAULT, loadRoutine(focus.DEFAULT));
      focusRoutines.put(focus.INTAKE, loadRoutine(focus.INTAKE));
      focusRoutines.put(focus.SHOOTER, loadRoutine(focus.SHOOTER));
      focusRoutines.put(focus.DRIVE, loadRoutine(focus.DRIVE));
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

                if (intake.getRollerSpeed().get() > 0.1) {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank.staggerGreen.blink(Seconds.of(0.5));
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

                if (launch.getVelocity().in(RPM) > 1) {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank
                            .staggerWhite
                            .scrollAtAbsoluteSpeed(
                                MetersPerSecond.of(launch.getVelocity().in(RadiansPerSecond) / 300),
                                kLedSpacing)
                            .overlayOn(
                                pBank.orange.synchronizedBlink(
                                    () -> {
                                      // Compares the live velocity to the optimal and flashes
                                      // orange if
                                      // they are not close.
                                      return !launch
                                          .getVelocity()
                                          .isNear(launch.getOptimalVelocity(), 0.02);
                                    }));
                      };
                } else {
                  level = priorityLevel.NORMAL_OPERATION;
                  returnPattern =
                      () -> {
                        return pBank.yellow;
                      };
                }

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
      return routineSupplier;
    }

    public LEDPattern update() {
      activePattern = focusRoutines.get(currentFocus).get().getPattern();
      return activePattern;
    }
  }
}
