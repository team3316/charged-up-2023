package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RollerGripperConstants;
import frc.robot.constants.SSDetectorConstants;

public class SSDetector extends SubsystemBase {
    private static final double ADC_RESOLUTION = 4096; // 12 bits
    private AnalogInput _detector;

    private Debouncer _debouncer = new Debouncer(0.1);

    public SSDetector() {
        _detector = new AnalogInput(SSDetectorConstants.id);
    }

    public boolean isAtSingleSubstation() {
        return getValue() > SSDetectorConstants.threshold;
    }

    public double getValue() {
        return this._detector.getValue() / ADC_RESOLUTION;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At SS", this.isAtSingleSubstation());
        SmartDashboard.putNumber("SS Value", this.getValue());
    }
}
