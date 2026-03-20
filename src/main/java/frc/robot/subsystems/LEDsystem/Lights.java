package frc.robot.subsystems.LEDsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LEDsystem.LEDpatternCache.LEDPatternCache;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is responsible for handling the LED display on the robot. It uses the 
 * AddressableLED class to control the lights, and the LEDPattern class to create various 
 * patterns and effects.
 * 
 * Periodically updates the LED pattern based on private conditions, which can be modified using setter methods
 * to reflect the robot's state, errors, or other information.
 */
public class Lights extends SubsystemBase {
  public int ticks = 0;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final LEDPatternCache p = new LEDPatternCache();
  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  
  private Time seconds(double s) {
    return Seconds.of(s);
  }

  /**
   * Helper function to create a flickering effect by alternating between a pattern and its reversed version
   * at a specified period. The pattern will switch every half period, creating a flickering effect.
   * @param pattern
   * @param period
   * @return pattern or pattern.reversed() depending on the current time and the specified period
   */
  private LEDPattern flicker(LEDPattern pattern, Time period){
    if (Timer.getTimestamp() % period.in(Seconds) < period.in(Seconds) / 2) {
      return pattern;
    } else {
      return pattern.reversed();
    }
  }

  /** Called once at the beginning of the robot program. */
  public Lights() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 120, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(120);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    ticks += 1;
    ticks %= 50000; // will (hopefully) limit ticks to 50000 and then reset it to 0
    // System.out.println(ticks);

    // Value for testing conditions
    int testVal = 0;

    // test conditions
    LEDPattern xColor = p.white;
    switch (testVal) {
      case 1:
        xColor = p.staggerRed.blink(seconds(1));
        break;
      case 2:
        xColor = flicker(p.staggerBlue, seconds(0.5));
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

/// Error codes (non-exhaustive)
/// Crimson - Colors are not returning, LEDs unusable
///   + Blink, Stagger functions are somehow broken
/// Solid White - No conditionals are being met (default color)
/// ...