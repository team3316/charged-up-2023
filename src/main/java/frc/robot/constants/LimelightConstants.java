package frc.robot.constants;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * LimelightConstants
 */
public class LimelightConstants {
    public static final double limelightHeightMeters = 0.27;
    public static final double lowGoalTargetHeightMeters = 0.62;
    public static final double highGoalTargetHeightMeters = 0;

    public static final double limeLightGoalHeightOffset = 0.38;
    public static final double limelightAngleFromFloorDegs = 20;

    public static final double installDistanceFromLowGoalMeters = 0.78;

    public static final Rotation2d installAngle = new Rotation2d(0);

    public static final List<String> actionEvents = Arrays.asList(new String[] { "arm", "manipulator" });

}
