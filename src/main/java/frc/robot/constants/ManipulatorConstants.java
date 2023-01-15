package frc.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ManipulatorConstants {

    public static final int sensorID = 0;

    public static final int leftSolenoidForwardChannel = 1;
    public static final int leftSolenoidReverseChannel = 2;
    public static final int rightSolenoidForwardChannel = 3;
    public static final int rightSolenoidReverseChannel = 4;

    public static final DoubleSolenoid.Value leftSolenoidOpenState = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value rightSolenoidOpenState = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value leftSolenoidCubeClosedState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value rightSolenoidCubeClosedState = DoubleSolenoid.Value.kOff;
    public static final DoubleSolenoid.Value leftSolenoidConeClosedState = DoubleSolenoid.Value.kOff;
    public static final DoubleSolenoid.Value rightSolenoidConeClosedState = DoubleSolenoid.Value.kForward;

}
