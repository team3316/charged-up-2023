package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.motors.PIDFGains;

public class RollerGripperConstants {
    public final static double kFoldingPercent = 0.3;

    public final static double kVoltage = 0;
    public final static double kms = 0;

    // TODO: update values when electronics table is compiled
    public final static int kTalonLeaderPort = 0;
    public final static int kTalonFollowerPort = 1;
    public final static int kSparkMaxFolderPort = 2;

    public final static double kFolderInValue = -0.3;
    public final static double kFolderOutValue = 0.3;
    public final static double kFolderOffValue = 0;

    public final static double kRollerIntakeValue = 0.70;
    public final static double kRollerEjectValue = -0.70;
    public final static double kRollerOffValue = 0;

    public final static double kFoldingSleepDuration = 0.05;
    public final static double kGrippingSleepDuration = 0.5;

    public final static double kEjectDuration = 0.02;

    public final static double kMaxFolderIn = 0;
    public final static double kMaxFolderOut = 90;

    // binding test
    public final static int kControllerPort = 1;

    // TODO: add values when calibrating the subsystem
    public static double kp, ki, kd, kf;
    public static PIDFGains folderGains = new PIDFGains(kp, ki, kd, kf);

    // pneumatics constants
    // TODO: update values when electronics table is compiled
    public final static int kSolenoidForwardChannel = 0;
    public final static int kSolenoidReverseChannel = 1;

    // TODO: update values when electronics table is compiled
    public final static DoubleSolenoid.Value kStateWhenFoldedIn = DoubleSolenoid.Value.kForward;
    public final static DoubleSolenoid.Value kStateWhenFoldedOut = DoubleSolenoid.Value.kReverse;
    public final static DoubleSolenoid.Value kStateWhenOff = DoubleSolenoid.Value.kOff;

}
