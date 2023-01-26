// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RollerGripperConstants;

public class AutoRollerGripper extends SubsystemBase {

    private TalonSRX _talonLeader, _talonFollower;
    private DigitalInput _rollerLimitSwitch;

    private DoubleSolenoid _doubleSolenoid;

    private TalonSRXConfiguration _talonConfig = new TalonSRXConfiguration();

    private FolderState _currentFolderState;
    private RollersState _currentRollerState;

    public enum FolderState {
        IN(RollerGripperConstants.stateWhenFoldedIn),
        OUT(RollerGripperConstants.stateWhenFoldedOut);

        private final DoubleSolenoid.Value pneumaticState;

        FolderState(DoubleSolenoid.Value pneumaticState) {
            this.pneumaticState = pneumaticState;
        }
    }

    public enum RollersState {
        INTAKE(RollerGripperConstants.rollerIntakePercent),
        EJECT(RollerGripperConstants.rollerEjectPercent),
        OFF(RollerGripperConstants.rollerOffPercent);

        private final double percentOutput;

        RollersState(double percentOutput) {
            this.percentOutput = percentOutput;
        }
    }

    public AutoRollerGripper() {
        _talonLeader = new TalonSRX(RollerGripperConstants.talonLeaderPort);
        _talonFollower = new TalonSRX(RollerGripperConstants.talonFollowerPort);
        _talonFollower.follow(_talonLeader);
        _talonFollower.setInverted(InvertType.OpposeMaster);

        _talonLeader.configAllSettings(_talonConfig);
        _talonFollower.configAllSettings(_talonConfig);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.solenoidForwardChannel,
                RollerGripperConstants.solenoidReverseChannel);

        _rollerLimitSwitch = new DigitalInput(RollerGripperConstants.rollerLimitSwitchPort);
    }

    public void periodic() {
        updateSDB();
    }

    public void updateSDB() {
        SmartDashboard.putBoolean("Auto Gripper Has Cone", hasCone());
    }

    public void setFolderState(FolderState state) {
        _doubleSolenoid.set(state.pneumaticState);

        SmartDashboard.putString("Auto Gripper State", _currentFolderState.toString());

        _currentFolderState = state;
    }

    public void setRollersState(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, state.percentOutput);
        _currentRollerState = state;
    }

    public RollersState getRollerState() {
        return _currentRollerState;
    }

    public void stop() {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, 0);
        _doubleSolenoid.set(Value.kOff);
    }

    public boolean hasCone() {
        return _rollerLimitSwitch.get();
    }

    public CommandBase getIntakeCommand() {
        return new StartEndCommand(
                () -> {
                    setRollersState(RollersState.INTAKE);
                },
                () -> {
                    new WaitCommand(RollerGripperConstants.intakeSleepDurationSeconds).andThen(
                            new InstantCommand(() -> setRollersState(RollersState.OFF)));
                }).until(this::hasCone);
    }

    public CommandBase getEjectCommand() {
        return new InstantCommand(() -> {
            setRollersState(RollersState.EJECT);
        }).andThen(new WaitCommand(RollerGripperConstants.ejectSleepDurationSeconds),
                new InstantCommand(() -> {
                    setRollersState(RollersState.OFF);
                }));
    }

    public CommandBase getFoldCommand(FolderState fState) {
        return new InstantCommand(() -> {
            setFolderState(fState);
        });
    }
}
