package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ManipulatorConstants {

    public static final int sensorID = 0;
    public static final int solenoidForwardChannel = 4;
    public static final int solenoidReverseChannel = 5;

    // TODO: update when piston position is known
    public static final DoubleSolenoid.Value solenoidOpenState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value solenoidClosedState = DoubleSolenoid.Value.kReverse;

    // ir constants
    public static final double CONEDetectorThreshold = 0.23;
    public static final double CONEDetectorHysteresis = 0.1;
    public static final double CUBEDetectorThreshold = 0.45;
    public static final double CUBEDetectorHysteresis = 0.1;
}
