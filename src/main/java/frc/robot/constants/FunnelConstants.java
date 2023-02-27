package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FunnelConstants {
    // TODO: update these values to real robot
    public static final DoubleSolenoid.Value openState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value closedState = DoubleSolenoid.Value.kReverse;

    public static final double collectPercent = -0.5;
    public static final double openPercent = 0;
    public static final double closedPercent = 0;

    public static final int talonSRXFollowerPort = 21;
    public static final int talonSRXLeaderPort = 20;
    public static final int solenoidForwardPort = 2;
    public static final int solenoidReversePort = 3;
}
