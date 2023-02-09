package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RollerGripperConstants {
    public final static int sparkMaxLeaderPort = 17;
    public final static int sparkMaxFollowerPort = 18;
    public final static int rollerLimitSwitchPort = 1;

    // TODO: calibrate values when system is built
    public final static double rollerIntakePercent = 0.70; // percent
    public final static double rollerEjectPercent = -0.70; // percent
    public final static double rollerOffPercent = 0; // percent

    public final static double ejectSleepDurationSeconds = 0.5; // seconds
    public final static double intakeSleepDurationSeconds = 0.5; // seconds

    public final static double ejectDurationSeconds = 0.02; // seconds
    

    public final static int solenoidForwardChannel = 4;
    public final static int solenoidReverseChannel = 5;
    // TODO: update when piston position is known
    public final static DoubleSolenoid.Value stateWhenFoldedIn = DoubleSolenoid.Value.kForward;
    public final static DoubleSolenoid.Value stateWhenFoldedOut = DoubleSolenoid.Value.kReverse;
    public final static DoubleSolenoid.Value stateWhenOff = DoubleSolenoid.Value.kOff;

}
