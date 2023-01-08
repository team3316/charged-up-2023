package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.motors.PIDFGains;

/**
 * DrivetrainConstants
 */
public class DrivetrainConstants {
    public static class SwerveModuleConstants {
        // TODO: update pidf gains
        public static final double driveKp = 0.612; // in seconds per meter
        public static final double driveKd = 0; // in seconds per meter
        public static final double driveKf = 0.75 / 2.81; // percent to motor / m/s at that
                                                          // percent
        public static final double steeringKp = 0.0124; // in 1 / wheel degrees

        private static final double neoMaxSpeed = 5600;
        private static final double driveRatio = 1.0 / 8.14;
        private static final double steeringRatio = 1.0 / 12.8;
        private static final double wheelDiameterMeters = 4 * 2.54 / 100; // 4 inches in meters

        public static final double drivePositionConversionFactor = driveRatio * wheelDiameterMeters * Math.PI; // m
                                                                                                               // /
                                                                                                               // rotation
        public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60; // m /
                                                                                                       // (rotation
                                                                                                       // *
                                                                                                       // seconds/minute)

        public static final double steeringPositionConversionFactor = steeringRatio * 360; // degrees / rotation
        public static final double steeringVelocityConversionFactor = steeringPositionConversionFactor / 60; // degrees
                                                                                                             // /
                                                                                                             // (rotation
                                                                                                             // *
                                                                                                             // seconds/minute)

        public static final double freeSpeedMetersPerSecond = neoMaxSpeed * driveVelocityConversionFactor;

        public final Translation2d position;
        public final int idDrive;
        public final PIDFGains driveGains = new PIDFGains(driveKp, 0, driveKd, driveKf);
        public final int idSteering;
        public final PIDFGains steeringGains = new PIDFGains(steeringKp);
        public final double cancoderZeroAngle;
        public final int canCoderId;

        public SwerveModuleConstants(
                Translation2d position,
                int idDrive,
                int idSteering,
                double cancoderZeroAngle,
                int canCoderId) {

            this.position = position;
            this.idDrive = idDrive;
            this.idSteering = idSteering;
            this.cancoderZeroAngle = cancoderZeroAngle;
            this.canCoderId = canCoderId;
        }
    }
}