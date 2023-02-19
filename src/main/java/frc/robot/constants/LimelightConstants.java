package frc.robot.constants;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.motors.PIDFGains;

/**
 * LimelightConstants
 */
public class LimelightConstants {
    public static final double AlignLimeLightToPigeonPhase = -1;

    public static final PIDFGains xGainsRetro = new PIDFGains(0.1, 0.0, 0.01, 0.0);
    public static final PIDFGains xGainsApril = new PIDFGains(0.1, 0.0, 0, 0.0);
    public static final double xTol = 1;

    public static final PIDFGains yGainsRetro = new PIDFGains(0.05, 0.0, 0.0, 0.0);
    public static final PIDFGains yGainsApril = new PIDFGains(0.05, 0.0, 0.0, 0.0);
    public static final double yTol = 0.5;

    public static final PIDFGains thetaGains = new PIDFGains(0.1, 0.0, 0.0, 0.0);
    public static final double thetaTol = 1;

    public static final double spinToleranceDegrees = 10;

    public static final Rotation2d limelightAxisToField = Rotation2d.fromDegrees(90);

    public static final double pipeLineRetroReflective = 0;
    public static final double pipeLineAprilTags = 1;

    public static final int LEDsByPipeline = 0;
    public static final int LEDsForceOff = 1;
    public static final int LEDsForceBlink = 2;
    public static final int LEDsForceON = 3;

    // TODO: these values are only approximation from PP
    public static final Translation2d collectionTranslation = new Translation2d(14.0, 7.5);
    public static final Rotation2d collectionHolonomicRotation = Rotation2d.fromDegrees(90);
    public static final Rotation2d collectionHeading = Rotation2d.fromDegrees(90);
    public static final PathPoint collectionPathPoint = new PathPoint(
            collectionTranslation,
            collectionHeading,
            collectionHolonomicRotation);

}
