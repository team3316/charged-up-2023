package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LightsConstants {

    public static final int port = 0;
    public static final int length = 10;
    public static final double blinkInterval = 0;

    public static enum RobotColorState {
        CUBE(new Color(0x66, 0x4D, 0xB3), Pattern.SOLID),
        CONE(new Color(0xEC, 0xDD, 0x0B), Pattern.SOLID),
        COLLECTED(Color.kOrange, Pattern.SOLID),
        // DEFAULT(Color.kOrangeRed, Pattern.SOLID),
        // DISABLED(null, Pattern.RAINBOW),
        OFF(Color.kBlack, Pattern.SOLID); // defualt color

        public final Color color;
        public final Pattern pattern;

        private RobotColorState(Color color, Pattern pattern) {
            this.color = color;
            this.pattern = pattern;
        }
    }

    public static enum Pattern {
        SOLID,
        BLINK,
        RAINBOW;
    }

}
