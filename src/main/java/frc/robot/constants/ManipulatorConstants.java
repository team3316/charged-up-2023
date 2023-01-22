package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ManipulatorConstants {

    public static final int sensorID = 0;

    public static final int solenoidForwardChannel = 2;
    public static final int solenoidReverseChannel = 3;

    // TODO: update when piston position is known
    public static final DoubleSolenoid.Value solenoidOpenState = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value solenoidClosedState = DoubleSolenoid.Value.kForward;

}
