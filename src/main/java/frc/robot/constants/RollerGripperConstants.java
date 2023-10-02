package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RollerGripperConstants {
    public final static int sparkMaxRightPort = 17;
    public final static int sparkMaxLeftPort = 18;
    public final static int rollerLimitSwitchPort = 1;

    public final static double rollerRightIntakePercent = +0.5; // percent
    public final static double rollerLeftIntakePercent = +0.3; // percent
    public final static double rollerEjectPercent = -0.7; // percent
    public final static double rollerOffPercent = 0; // percent

    public final static double ejectSleepDurationSeconds = 0.5; // seconds
    public final static double intakeSleepDurationSeconds = 0.15; // seconds

    public final static double switchDebounceTimeSecs = 0.1;

    public final static int currentLimitAmp = 10; // in Amps

    public final static int solenoidForwardChannel = 0;
    public final static int solenoidReverseChannel = 1;
    public final static DoubleSolenoid.Value stateWhenFoldedIn = DoubleSolenoid.Value.kReverse;
    public final static DoubleSolenoid.Value stateWhenFoldedOut = DoubleSolenoid.Value.kForward;
    public final static DoubleSolenoid.Value stateWhenOff = DoubleSolenoid.Value.kOff;

}
