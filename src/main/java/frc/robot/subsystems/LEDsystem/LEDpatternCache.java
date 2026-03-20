package frc.robot.subsystems.LEDsystem;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDpatternCache {

    public static final class LEDPatternCache {
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

        // Lights off
        public final LEDPattern off = LEDPattern.kOff;
    }
}
