package frc.robot.commands.calibrations;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmCalibration extends CommandBase {

    PS4Controller _controller;

    ArmFeedforward _feedforward;

    Subsystem _subsystem;
    TalonFX _leader;

    boolean isEnded = false;

    double _currentValue;

    public ArmCalibration(Arm subsystem, PS4Controller controller) {
        this._subsystem = subsystem;
        this._leader = subsystem.getLeader();
        this._controller = controller;
    }

    public void init() {
        SmartDashboard.putBoolean("is calibration activated", false);
        SmartDashboard.putNumber("arm voltage calibration", 0);
    }

    public void execute() {
        _currentValue = SmartDashboard.getNumber("arm voltage calibration", 0);
        _leader.set(TalonFXControlMode.PercentOutput, _currentValue);
        SmartDashboard.putNumber("arm voltage calibration", _currentValue);
    }

    public boolean isFinished() {
        if (_controller.getCircleButtonPressed())
            return true;

        return false;
    }

    public void end() {
        _leader.set(TalonFXControlMode.PercentOutput, 0);
    }
}
