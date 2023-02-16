// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FunnelConstants;

public class Funnel extends SubsystemBase {

    private TalonSRX _followerRoller;
    private TalonSRX _leaderRoller;
    private FunnelPosition _currentPosition;
    private FunnelRollersState _currentRollersState;
    private DoubleSolenoid _funnelSolenoid;

    public enum FunnelPosition {
        OPEN(FunnelConstants.openState),
        CLOSED(FunnelConstants.closedState);

        public final DoubleSolenoid.Value solenoidState;

        private FunnelPosition(DoubleSolenoid.Value solenoidState) {
            this.solenoidState = solenoidState;
        }
    }

    public enum FunnelRollersState {
        COLLECT(FunnelConstants.collectPercent),
        OFF(FunnelConstants.closedPercent);

        public double rollersPrecent;

        private FunnelRollersState(double rollerPercent) {
            this.rollersPrecent = rollerPercent;
        }
    }

    public Funnel() {
        this._funnelSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, FunnelConstants.solenoidForwardPort,
                FunnelConstants.solenoidReversePort);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        talonConfig.continuousCurrentLimit = 5; // Low torque application
        talonConfig.openloopRamp = 1.0; // Seconds from 0 to 100%
        talonConfig.voltageCompSaturation = 12; // For consistency

        this._followerRoller = new TalonSRX(FunnelConstants.talonSRXFollowerPort);
        this._leaderRoller = new TalonSRX(FunnelConstants.talonSRXLeaderPort);
        _followerRoller.configFactoryDefault();
        _leaderRoller.configFactoryDefault();
        _followerRoller.configAllSettings(talonConfig);
        _leaderRoller.configAllSettings(talonConfig);

        _followerRoller.follow(_leaderRoller);
        _followerRoller.setInverted(InvertType.OpposeMaster);

    }

    public void stop() {
        _funnelSolenoid.set(DoubleSolenoid.Value.kOff);
        this.setFunnelRollersState(FunnelRollersState.OFF);
    }

    public FunnelPosition getFunnelPosition() {
        return _currentPosition;
    }

    public FunnelRollersState getFunnelRollersState() {
        return _currentRollersState;
    }

    private void setFunnelPosition(FunnelPosition targetPosition) {
        if (_currentPosition == targetPosition) {
            return;
        }

        _funnelSolenoid.set(targetPosition.solenoidState);

        _currentPosition = targetPosition;

        SmartDashboard.putString("Funnel Position", targetPosition.toString());
    }

    private void setFunnelRollersState(FunnelRollersState targetState) {
        if (_currentRollersState == targetState) {
            return;
        }

        _leaderRoller.set(TalonSRXControlMode.PercentOutput, targetState.rollersPrecent);

        _currentRollersState = targetState;

        SmartDashboard.putString("Funnel Rollers State", targetState.toString());
    }

    public CommandBase setFunnelPositionCommand(FunnelPosition targetPositon) {
        return new InstantCommand(() -> setFunnelPosition(targetPositon), this);
    }

    public CommandBase setFunnelRollersStateCommand(FunnelRollersState targetState) {
        return new InstantCommand(() -> setFunnelRollersState(targetState), this);
    }
}
