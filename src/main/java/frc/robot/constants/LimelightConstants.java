package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * LimelightConstants
 */
public class LimelightConstants {
    public static final double AlignLimeLightToPigeonPhase = -1;

    public static final double xKP = 0.05;
    public static final double xKI = 0.0;
    public static final double xKD = 0.0;
    public static final double xTol = 1;

    public static final double yKP = 0.06;
    public static final double yKI = 0.0;
    public static final double yKD = 0.0;
    public static final double yTol = 0.5;

    public static final double thetaKP = 0.05;
    public static final double thetaKI = 0.0;
    public static final double thetaKD = 0.0;
    public static final double thetaTol = 1;

    public static final Rotation2d limelightRotations = new Rotation2d(-Math.PI * 0.5);

    public static final Rotation2d installAngle = Rotation2d.fromDegrees(-90);

}
