package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.motors.PIDFGains;

/**
 * LimelightConstants
 */
public class LimelightConstants {
    public static final double AlignLimeLightToPigeonPhase = -1;

    public static final PIDFGains xGains = new PIDFGains(0.05, 0.0, 0.0, 0.0);
    public static final double xTol = 1;

    public static final PIDFGains yGains = new PIDFGains(0.06, 0.0, 0.0, 0.0);
    public static final double yTol = 0.5;

    public static final PIDFGains thetaGains = new PIDFGains(0.05, 0.0, 0.0, 0.0);
    public static final double thetaTol = 1;

    public static final double spinToleranceDegrees = 10;

    public static final Rotation2d limelightAxisToField = Rotation2d.fromDegrees(90);

    public static final double pipeLineRetroReflective = 0;
    public static final double pipeLineAprilTags = 1;

}
