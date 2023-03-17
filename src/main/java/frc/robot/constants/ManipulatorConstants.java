package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ManipulatorConstants {

    public static final int sensorID = 0;
    public static final int solenoidForwardChannel = 4;
    public static final int solenoidReverseChannel = 5;

    // TODO: update when piston position is known
    public static final DoubleSolenoid.Value solenoidOpenState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value solenoidClosedState = DoubleSolenoid.Value.kReverse;

    /*
     * No game piece, funnel open = 0.05
     * No game piece, funnel closed = 0.075
     * cube, open = 0.35-0.48
     * cube, closed = 0.45-0.6
     * cone, open = 0.21-0.26
     * cone, closed = 0.3
     */
    // ir constants
    public static final double FunnelingDetectorThreshold = 0.12;
    public static final double FunnelingDetectorHysteresis = 0.0;
    public static final double CONEDetectorThreshold = 0.23;
    public static final double CONEDetectorHysteresis = 0.0;
    public static final double CUBEDetectorThreshold = 0.35;
    public static final double CUBEDetectorHysteresis = 0.0;
}
