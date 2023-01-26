package frc.robot.commands.calibrations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.humanIO.CommandPS5Controller;
import frc.robot.subsystems.Arm;

public class ArmCalibration extends CommandBase {

    CommandPS5Controller _controller;

    ArmFeedforward _feedforward;

    Subsystem _subsystem;
    TalonFX _leader;
    TalonFXConfiguration _leaderConfig;

    boolean calibrationSequence = true;

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

    public ArmCalibration(Arm subsystem, CommandPS5Controller controller) {
        _subsystem = subsystem;
        _leader = subsystem.getLeader();
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
        SmartDashboard.putNumber("current arm percent", 0);

        SmartDashboard.putString("pid new gain", "new gain");
        SmartDashboard.putNumber("pid new value", 0);
        SmartDashboard.putData("add to pid", new InstantCommand(() -> addToPID(
                SmartDashboard.getString("pid new gain", "new gain"),
                SmartDashboard.getNumber("pid new value", 0))));

        SmartDashboard.putString("ff new gain", "new gain");
        SmartDashboard.putNumber("ff new value", 0);
        SmartDashboard.putData("add to pid", new InstantCommand(() -> addToFeedforward(
                SmartDashboard.getString("ff new gain", "new gain"),
                SmartDashboard.getNumber("ff new value", 0))));
    }

    public void execute() {
        _currentValue = SmartDashboard.getNumber("current arm percent", 0);
        _leader.set(ControlMode.PercentOutput, _currentValue);
        SmartDashboard.putNumber("current arm percent", _currentValue);
    }

    public boolean isFinished() {
        return !calibrationSequence;
    }

    public CommandBase endCalibrationSequence() {
        return new InstantCommand(() -> calibrationSequence = false);
    }

    public void end() {
        _leader.set(TalonFXControlMode.PercentOutput, 0);
    }
}
