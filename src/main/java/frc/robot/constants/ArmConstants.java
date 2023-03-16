package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
    // Arm motors
    public static final int leaderCANID = 14;
    public static final int followerCANID = 15;

    // Motion profile
    public static final double maxFastVelocityDegreesPerSec = 350; // in deg/s
    public static final double maxFastAccelerationDegreesPerSecSqrd = 650; // in
    // deg/s^2

    public static final double maxSlowVelocityDegreesPerSec = 350; // in deg/s
    public static final double maxSlowAccelerationDegreesPerSecSqrd = 650; // in
    // deg/s^2
    public static final Constraints fastTrapezoidConstraints = new Constraints(maxFastVelocityDegreesPerSec,
            maxFastAccelerationDegreesPerSecSqrd);

    public static final Constraints slowTrapezoidConstraints = new Constraints(maxSlowVelocityDegreesPerSec,
            maxSlowAccelerationDegreesPerSecSqrd);

    // Arm gains
    public static final double kP = 0.02;
    public static final double kMaxOutput = 0.8;

    // Arm feedforward
    public static final double gravityGain = 0.017; // in Motor%
    public static final double velocityGain = 0.3 / Math.toRadians(104); // in Motor% sec/deg
    public static final double staticGain = 0.009; // in Motor%
    public static final double accelerationGain = 0; // in Motor% sec^2/deg

    // State Angles
    // TODO: Get exact numbers from Guy
    public static final double collectAngle = -54; // in deg
    public static final double driveAngle = 70; // in deg
    public static final double midCubeAngle = 150; // in deg
    public static final double midConeAngle = 150; // in deg
    public static final double lowAngle = 100; // in deg
    public static final double outOfFunnelAngle = -20; // in deg

    public static final double gearRatio = 1.0 / 120.0;

}
