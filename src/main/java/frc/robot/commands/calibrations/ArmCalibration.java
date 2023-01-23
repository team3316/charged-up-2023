package frc.robot.commands.calibrations;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmCalibration extends CommandBase {

    PS4Controller _controller;

    ArmFeedforward _feedforward;

    Subsystem _subsystem;
    TalonFX _leader;
    TalonFXConfiguration _leaderConfig;

    // ff gains
    double ks = 0;
    double kv = 0;
    double kg = 0;
    double ka = 0;

    // pid gains
    double kp = 0;
    double ki = 0;
    double kd = 0;

    boolean isEnded = false;

    double _currentValue;

    public ArmCalibration(Arm subsystem, PS4Controller controller) {
        this._subsystem = subsystem;
        this._leader = subsystem.getLeader();
        this._controller = controller;

        this._leader.configAllSettings(_leaderConfig);
    }

    public void addToFeedforward(String gain, double value) {
        gain.toLowerCase();
        switch (gain) {
            case "ks":
                ks = value;
                break;
            case "kv":
                kv = value;
                break;
            case "kg":
                kg = value;
                break;
            case "ka":
                ka = value;
                break;

        }
        _feedforward = new ArmFeedforward(ks, kv, kg, ka);
    }

    public void addToPID(String gain, double value) {
        gain.toLowerCase();
        switch (gain) {
            case "kp":
                _leaderConfig.slot0.kP = value;
                break;
            case "ki":
                _leaderConfig.slot1.kI = value;
                break;
            case "kd":
                _leaderConfig.slot2.kD = value;
                break;
        }
    }

    public void init() {
        SmartDashboard.putBoolean("is calibration activated", false);
        SmartDashboard.putNumber("arm voltage calibration", 0);

        SmartDashboard.putString("pid new gain", "new gain");
        SmartDashboard.putNumber("pid new value", 00);
        SmartDashboard.putData("add to pid", new InstantCommand(() -> addToPID(
                SmartDashboard.getString("pid new gain", "new gain"),
                SmartDashboard.getNumber("pid new value", 00))));
    }

    public void execute() {
        _currentValue = SmartDashboard.getNumber("arm voltage calibration", 0);
        _leader.set(TalonFXControlMode.Position, _currentValue);
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
