package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutonomousConstants {
    // TODO: update speeds
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 5;

    public static final String defaultPath = "rotate";

    // TODO: update P gains
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 3;

    public static final Constraints kThetaControllerConstraints = new Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}
