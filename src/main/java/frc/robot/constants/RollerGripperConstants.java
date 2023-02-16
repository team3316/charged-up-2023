package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RollerGripperConstants {
    public final static int sparkMaxTopPort = 17;
    public final static int sparkMaxBottomPort = 18;
    public final static int rollerLimitSwitchPort = 1;

    public final static double rollerTopIntakePercent = +0.5; // percent
    public final static double rollerBottomIntakePercent = +0.3; // percent
    public final static double rollerEjectPercent = -0.7; // percent
    public final static double rollerOffPercent = 0; // percent

    public final static double ejectSleepDurationSeconds = 0.5; // seconds
    public final static double intakeSleepDurationSeconds = 0.15; // seconds

    public final static int currentLimitAmp = 20; // in Amps

    public final static int solenoidForwardChannel = 4;
    public final static int solenoidReverseChannel = 5;
    public final static DoubleSolenoid.Value stateWhenFoldedIn = DoubleSolenoid.Value.kReverse;
    public final static DoubleSolenoid.Value stateWhenFoldedOut = DoubleSolenoid.Value.kForward;
    public final static DoubleSolenoid.Value stateWhenOff = DoubleSolenoid.Value.kOff;

}
