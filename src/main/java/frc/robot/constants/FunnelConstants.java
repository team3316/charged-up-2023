package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FunnelConstants {
    public static DoubleSolenoid.Value collectState = DoubleSolenoid.Value.kForward;
    public static DoubleSolenoid.Value installState = DoubleSolenoid.Value.kForward;
    public static DoubleSolenoid.Value closedState = DoubleSolenoid.Value.kReverse;

    public static double collectSpeed = -1;
    public static double installSpeed = 0;
    public static double closedSpeed = 0;

}
