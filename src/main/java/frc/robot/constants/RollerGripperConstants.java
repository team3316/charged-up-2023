package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.motors.PIDFGains;

public class RollerGripperConstants {
    // TODO: update values when electronics table is compiled
    public final static int talonLeaderPort = 0;
    public final static int talonFollowerPort = 1;

    public final static double rollerIntakeValue = 0.70; // percent
    public final static double rollerEjectValue = -0.70; // percent
    public final static double rollerOffValue = 0; // percent

    public final static double foldingSleepDuration = 0.05; // seconds
    public final static double grippingSleepDuration = 0.5; // seconds

    public final static double ejectDuration = 0.02; // seconds

    // spark max folder
    public final static int sparkMaxFolderPort = 2;
    // TODO: add values when calibrating the subsystem
    public static double kp = 0.01; // motor%/deg
    public static PIDFGains folderGains = new PIDFGains(kp);
    public static double inAngle = 0;
    public static double outAngle = 90;
    public static double gearRatio = 1;
    public static double positionConversionFactor = gearRatio * 360; // converts position to degrees
    public static double velocityConversionFactor = 1;
    

    // pneumatics folder
    // TODO: update values when electronics table is compiled
    public final static int solenoidForwardChannel = 0;
    public final static int solenoidReverseChannel = 1;
    public final static DoubleSolenoid.Value stateWhenFoldedIn = DoubleSolenoid.Value.kForward;
    public final static DoubleSolenoid.Value stateWhenFoldedOut = DoubleSolenoid.Value.kReverse;
    public final static DoubleSolenoid.Value stateWhenOff = DoubleSolenoid.Value.kOff;

}
