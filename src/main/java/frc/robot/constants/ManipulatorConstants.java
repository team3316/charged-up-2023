package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ManipulatorConstants {

    public static final int sensorID = 0;
    public static final int solenoidForwardChannel = 4;
    public static final int solenoidReverseChannel = 5;

    // TODO: update when piston position is known
    public static final DoubleSolenoid.Value solenoidOpenState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value solenoidClosedState = DoubleSolenoid.Value.kReverse;

    public static final double GPDetectorThreshold = 0.3;
    public static final double GPDetectorHysteresis = 0.1;
}
