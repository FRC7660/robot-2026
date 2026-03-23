package frc.robot.subsystems.LEDsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDsystem.LEDPatternManager.LightRoutine;
import frc.robot.subsystems.LEDsystem.LEDPatternManager.PatternBank;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is responsible for handling the LED display on the robot. It uses the AddressableLED
 * class to control the lights, and the LEDPattern class to create various patterns and effects.
 *
 * <p>Periodically updates the LED pattern based on private conditions, which can be modified using
 * setter methods to reflect the robot's state, errors, or other information.
 */
public class Lights extends SubsystemBase {
  private final AddressableLED m_led;
  private static LightRoutine activeRoutine;
  private LEDPattern xColor;
  private final AddressableLEDBuffer m_ledBuffer;
  private final PatternBank p = new PatternBank();
  private final LEDPatternManager patternManager;
  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = LEDPatternManager.kLedSpacing;

  private Time seconds(double s) {
    return Seconds.of(s);
  }

  /**
   * Helper function to create a flickering effect by alternating between a pattern and its reversed
   * version at a specified period. The pattern will switch every half period, creating a flickering
   * effect.
   *
   * @param pattern
   * @param period
   * @return pattern or pattern.reversed() depending on the current time and the specified period
   */

  /** Called once at the beginning of the robot program. */
  public Lights(
      Launch launchSystem, Intake intakeSystem, SwerveSubsystem swerveSystem, Turret turretSystem) {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);
    xColor = p.red;

    // Instantiate the LED pattern manager, which will handle the logic for determining which
    // pattern to display based on the robot's state
    patternManager = new LEDPatternManager();

    // Instantiate Routine
    activeRoutine = new LightRoutine(launchSystem, intakeSystem, swerveSystem, turretSystem);

    // Reuse buffer
    // Default to a length of 120, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(120);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // /** This function assigns the lights a new routine */
  // public void setRoutine(LightRoutine routine) {
  //   activeRoutine = routine;
  // }

  @Override
  public void periodic() {
    xColor = activeRoutine.update();

    // Value for testing conditions (overrides when above 0)
    int testVal = 0;

    // test conditions
    switch (testVal) {
      case 1:
        xColor = p.staggerRed.blink(seconds(1));
        break;
      case 2:
        xColor = p.flickerBlue.get();
        break;
      case 3:
        xColor = p.green.blink(seconds(1));
        break;
      case 4:
        xColor = p.rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing);
        break;
      case 5:
        xColor = p.off;
        break;
      default:
        // keep default xColor (m_white)
        break;
    }

    // System.out.println(xColor); // Use this print function if LEDs are not currently testable
    xColor.applyTo(m_ledBuffer);

    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }
}
