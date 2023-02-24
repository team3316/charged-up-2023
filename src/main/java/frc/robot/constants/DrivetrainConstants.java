package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final double maxRotationSpeedRadPerSec = 11.5;

    public static final double frontWheelDistMeters = 0.4803;
    public static final double sideWheelDistMeters = 0.6703;

    public final static double cancoderTLOffset = 33.2;
    public final static double cancoderTROffset = 324.0;
    public final static double cancoderBLOffset = 182.8;
    public final static double cancoderBROffset = 110.4;
    public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(
            new Translation2d(sideWheelDistMeters / 2, frontWheelDistMeters / 2), 3, 4,
            cancoderTLOffset, 11);

    public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(
            new Translation2d(sideWheelDistMeters / 2, -frontWheelDistMeters / 2), 1, 2,
            cancoderTROffset, 10);

    public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(
            new Translation2d(-sideWheelDistMeters / 2, frontWheelDistMeters / 2), 5, 6,
            cancoderBLOffset, 12);

    public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(
            new Translation2d(-sideWheelDistMeters / 2, -frontWheelDistMeters / 2), 7, 8,
            cancoderBROffset, 13);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
            TLModule.position, BRModule.position, BLModule.position);

    public static final int pigeonId = 9;

    public static final Rotation2d installAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d collectAngle = Rotation2d.fromDegrees(90);

}