package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FunnelConstants {
    // TODO: update these values to real robot
    public static final DoubleSolenoid.Value openState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value closedState = DoubleSolenoid.Value.kReverse;

    public static final double collectPercent = -0.15;
    public static final double installPercent = 0;
    public static final double closedPercent = 0;

    public static final int sparkMaxPort = 20;
    public static final int solenoidForwardPort = 1;
    public static final int solenoidReversePort = 2;

}
