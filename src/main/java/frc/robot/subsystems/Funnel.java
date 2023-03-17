// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private FunnelState _currentState;
    private DoubleSolenoid _funnelSolenoid;

    public enum FunnelState {
        COLLECT(FunnelConstants.openState, FunnelConstants.collectPercent),
        KEEPIN(FunnelConstants.closedState, FunnelConstants.keepinPercent),
        OPEN(FunnelConstants.openState, FunnelConstants.openPercent),
        READJUST(FunnelConstants.closedState, FunnelConstants.collectPercent),
        CLOSED(FunnelConstants.closedState, FunnelConstants.closedPercent),
        EJECT(FunnelConstants.closedState, FunnelConstants.ejectPercent);

        public final DoubleSolenoid.Value solenoidState;
        public final double rollerPercent;

        private FunnelState(Value solenoidState, double rollerPercent) {
            this.solenoidState = solenoidState;
            this.rollerPercent = rollerPercent;
        }
    };

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
        _leaderRoller.setNeutralMode(NeutralMode.Coast);

        _followerRoller.follow(_leaderRoller);
        _followerRoller.setInverted(InvertType.OpposeMaster);
        _followerRoller.setNeutralMode(NeutralMode.Coast);

    }

    public void stop() {
        _funnelSolenoid.set(DoubleSolenoid.Value.kOff);
        _leaderRoller.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public FunnelState getFunnelState() {
        return _currentState;
    }

    private void setFunnelState(FunnelState targetState) {
        if (_currentState == targetState) {
            return;
        }

        _funnelSolenoid.set(targetState.solenoidState);
        _leaderRoller.set(TalonSRXControlMode.PercentOutput, targetState.rollerPercent);

        _currentState = targetState;

        SmartDashboard.putString("Funnel State", targetState.toString());
    }

    public CommandBase setFunnelStateCommand(FunnelState state) {
        return new InstantCommand(() -> setFunnelState(state), this).withTimeout(0.1);
    }
}
