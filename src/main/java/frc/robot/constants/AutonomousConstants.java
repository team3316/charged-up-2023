package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutonomousConstants {
    // TODO: update .pathplanner/settings.json
    // TODO: update speeds
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;

    public static final String defaultPath = "forward";

    // TODO: update P gains
    public static final double kPTranslationController = 1;
    public static final double kPThetaController = 3;
}
