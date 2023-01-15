package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FunnelConstants {
    public static final DoubleSolenoid.Value collectState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value installState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value closedState = DoubleSolenoid.Value.kReverse;

    public static final double collectSpeed = -1;
    public static final double installSpeed = 0;
    public static final double closedSpeed = 0;

}
