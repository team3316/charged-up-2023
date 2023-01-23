package frc.robot.commands.calibrations;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmCalibration extends CommandBase {

    Subsystem _subsystem;
    TalonFX _leader;

    double _currentValue;

    enum CalibrationStates {
        ACTIVE,
        DISABLED
    }

    CalibrationStates _currentState;

    public ArmCalibration(Arm subsystem) {
        this._subsystem = subsystem;
        this._leader = subsystem.getLeader();
    }

    public void init() {
        SmartDashboard.putBoolean("is calibration activated", false);
        SmartDashboard.putNumber("arm voltage calibration", 0);
    }

    public void execute() {
        _currentState = (SmartDashboard.getBoolean("is calibration activated", false)) ? CalibrationStates.ACTIVE
                : CalibrationStates.DISABLED;
        _currentValue = SmartDashboard.getNumber("arm voltage calibration", 0);
        if (_currentState == CalibrationStates.ACTIVE)
            _leader.set(TalonFXControlMode.PercentOutput, _currentValue);

        SmartDashboard.putNumber("arm voltage calibration", _currentValue);
    }

}
