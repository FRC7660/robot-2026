package frc.robot.subsystems.LEDsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import java.util.Map;
import java.util.TreeMap;
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

    // Staggered colors
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

    // Flickering colors
    private LEDPattern flicker(LEDPattern pattern, Time period) {
      if (Timer.getTimestamp() % period.in(Seconds) < period.in(Seconds) / 2) {
        return pattern;
      } else {
        return pattern.reversed();
      }
    }

    public final LEDPattern flickerRed = flicker(staggerRed, Seconds.of(0.5));
    public final LEDPattern flickerYellow = flicker(staggerYellow, Seconds.of(0.5));
    public final LEDPattern flickerGreen = flicker(staggerGreen, Seconds.of(0.5));
    public final LEDPattern flickerBlue = flicker(staggerBlue, Seconds.of(0.5));
    public final LEDPattern flickerPurple = flicker(staggerPurple, Seconds.of(0.5));
    public final LEDPattern flickerOrange = flicker(staggerOrange, Seconds.of(0.5));
    public final LEDPattern flickerWhite = flicker(staggerWhite, Seconds.of(0.5));

    // Lights off
    public final LEDPattern off = LEDPattern.kOff;
  }

  public static class LightRoutine {
    private PatternBank p = new PatternBank();
    public LEDPattern value;
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
    };

    TreeMap<focus, Supplier<LEDPattern>> focusRoutines = new TreeMap<>();

    /** Initialization */
    public LightRoutine(
        Launch launchSystem,
        Intake intakeSystem,
        SwerveSubsystem swerveSystem,
        Turret turretSystem) {
      LEDPattern value = LEDPattern.kOff;
      currentFocus = focus.DEFAULT;
      pBank = new PatternBank();

      // Subsystem assignment for easier access in routines
      launch = launchSystem;
      intake = intakeSystem;
      drivebase = swerveSystem;
      turret = turretSystem;

      // Default/Test routines
      focusRoutines.put(focus.DEFAULT, getRoutine("Test", focus.DEFAULT));
      focusRoutines.put(focus.DEFAULT, getRoutine("Rainbow", focus.DEFAULT));

      // Subsystem-specific routines
      focusRoutines.put(focus.INTAKE, getRoutine("Test", focus.INTAKE));
      focusRoutines.put(focus.SHOOTER, getRoutine("Test", focus.SHOOTER));
      focusRoutines.put(focus.DRIVE, getRoutine("Test", focus.DRIVE));
    }

    public Supplier<LEDPattern> getRoutine(String routineName, focus focusName) {
      Supplier<LEDPattern> returnPattern = () -> LEDPattern.kOff;

      switch (focusName) {
        case DEFAULT:
          returnPattern =
              () -> {
                if (routineName == "Rainbow") {
                  return pBank.rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
                }
                return pBank.white;
              };
          break;

        case INTAKE:
          returnPattern =
              () -> {
                if (routineName == "Test") {
                  return pBank.staggerGreen.blink(Seconds.of(0.5));
                }
                return pBank.green;
              };
          break;

        case SHOOTER:
          returnPattern =
              () -> {
                if (routineName == "Test") {
                  return pBank.staggerRed.blink(Seconds.of(0.5));

                } else if (routineName == "MuzzleFlash") {
                  return pBank
                      .staggerWhite
                      .scrollAtAbsoluteSpeed(
                          MetersPerSecond.of(launch.getVelocity().in(RadiansPerSecond) / 300),
                          kLedSpacing)
                      .overlayOn(
                          pBank.orange.synchronizedBlink(
                              () -> {
                                // Compares the live velocity to the optimal and flashes orange if
                                // they are not close.
                                return !launch
                                    .getVelocity()
                                    .isNear(launch.getOptimalVelocity(), 0.02);
                              }));
                }
                ;
                return pBank.yellow;
              };
          break;

        case DRIVE:
          returnPattern =
              () -> {
                if (routineName == "Test") {
                  return pBank.staggerBlue.blink(Seconds.of(0.5));

                } else if (routineName == "ErrorDisplay") {
                  if (drivebase.getSwerveDrive().getGyro().getAccel().isEmpty()) {
                    // If the gyro is not providing acceleration data, it is likely disconnected or
                    // malfunctioning,
                    // so the LEDs will blink red as an error signal.
                    return pBank.red.breathe(Seconds.of(1));
                  } else if (true) {
                    // Placeholder for another error condition, such as a disconnected motor.
                    // This can be updated with an actual condition once the drivebase code is
                    // implemented.
                    return pBank.orange.breathe(Seconds.of(1));
                  }
                }
                return pBank.blue;
              };
          break;

        default:
          break;
      }
      return returnPattern;
    }

    public void update() {}

    public LEDPattern testRoutine(LEDPattern basePattern) {
      return basePattern;
    }

    public LEDPattern get() {
      return value;
    }
  }
}
