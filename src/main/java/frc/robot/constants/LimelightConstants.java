package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * LimelightConstants
 */
public class LimelightConstants {
    public static final double AlignLimeLightToPigeonPhase = -1;

    public static final double xKp = 2.0; // in 1/sec: 1 meter error moves in 2.0 meter/seconds
    public static final double xTol = 0.01; // in meters
    public static final double yKp = 2.0; // in 1/sec: 1 meter error moves in 2.0 meter/seconds
    public static final double yTol = 0.01; // in meters
    public static final double thetaKp = 0.1; // in rad/degree*sec: 1 degree error moves in 0.1 rad/sec
    public static final double thetaTol = 1; // in degrees

    public static class AutoAlignConstants {
        public double xGoal, hDiff;

        public AutoAlignConstants(double xGoal, double hDiff) {
            this.xGoal = xGoal;
            this.hDiff = hDiff;
        }
    }

    public static final AutoAlignConstants cone = new LimelightConstants.AutoAlignConstants(0.7, 0.42);
    public static final AutoAlignConstants cube = new LimelightConstants.AutoAlignConstants(0.48, 0.21);

    public static final double spinToleranceDegrees = 10;

    public static final Rotation2d limelightAxisToField = Rotation2d.fromDegrees(90);

    public static final double pipeLineRetroReflective = 0;
    public static final double pipeLineAprilTags = 1;

    public static final int LEDsByPipeline = 0;
    public static final int LEDsForceOff = 1;
    public static final int LEDsForceBlink = 2;
    public static final int LEDsForceON = 3;
}
