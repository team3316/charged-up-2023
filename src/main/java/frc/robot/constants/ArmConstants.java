package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.motors.PIDFGains;

public class ArmConstants {
    // Arm motors
    public static final int leaderCANID = 16;
    public static final int followerCANID = 17;

    public static final boolean motorInverted = false;

    public static final double motorToArmConversionFactor = 360 / 33.6; // in Degrees: 360 degrees / gear
                                                                        // reduction
    public static final double keepPrecent = -0.1;

    // Arm motion
    public static final double overshootDelta = 1;
    public static final double intakeAngle = 36 + overshootDelta; // in Degrees: measured. Theoretical was -37.6
    public static final double shootAngle = -118; // in Degrees: measured. Theoretical was 119.6

    public static final double startingAngle = -90;

    // motion profile.
    public static final double movementTime = 1.75; // in secs.
    public static final double maxVelocityDegreesPerSec = 180 * 2 / movementTime; // in Degrees/s
    public static final double maxAccelerationDegreesPerSecSqrd = maxVelocityDegreesPerSec / (movementTime / 2); // in
                                                                                                                 // Degrees/s^2
    public static final Constraints trapezoidConstraints = new Constraints(maxVelocityDegreesPerSec,
            maxAccelerationDegreesPerSecSqrd);

    // Arm gains
    public static final double kP = 0.03;
    public static final double kMaxOutput = 0.4;
    public static final PIDFGains gains = new PIDFGains(kP, 0, 0, 0, kMaxOutput);

    // Arm feedforward
    public static final double gravityGain = -0.1; // in Motor%
    public static final double velocityGain = 0.000965; // in Motor% sec/deg
    public static final double staticGain = 0.02; // in Motor%
    public static final double accelerationGain = 0; // in Motor% sec^2/deg

    // State Angles - need to get exact numbers from Guy
    public static final double collectAngle = 250; // in deg
    public static final double driveAngle = 90; // in deg
    public static final double midCubeAngle = 30; // in deg
    public static final double midConeAngle = 30; // in deg
    public static final double lowAngle = -50; // in deg

}
