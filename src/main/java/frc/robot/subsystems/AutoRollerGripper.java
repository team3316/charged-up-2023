// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RollerGripperConstants;
import frc.robot.motors.DBugSparkMax;

public class AutoRollerGripper extends SubsystemBase {

    private TalonSRX _talonLeader, _talonFollower;
    private DigitalInput _rollerLimitSwitch;

    private DBugSparkMax _folderSM;

    private DoubleSolenoid _doubleSolenoid;

    private TalonSRXConfiguration _talonConfig = new TalonSRXConfiguration();

    private FolderState currentFolderState;

    public enum FolderState {
        IN(RollerGripperConstants.inAngle, RollerGripperConstants.stateWhenFoldedIn),
        OUT(RollerGripperConstants.outAngle, RollerGripperConstants.stateWhenFoldedOut);

        private final double desiredAngle;
        private final DoubleSolenoid.Value pneumaticState;

        FolderState(double desiredAngle, DoubleSolenoid.Value pneumaticState) {
            this.desiredAngle = desiredAngle;
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

        _folderSM = DBugSparkMax.create(RollerGripperConstants.sparkMaxFolderPort, RollerGripperConstants.folderGains,
                RollerGripperConstants.positionConversionFactor, RollerGripperConstants.velocityConversionFactor,
                RollerGripperConstants.inAngle);

        _folderSM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        _folderSM.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);

        _folderSM.getEncoder().setPositionConversionFactor(RollerGripperConstants.positionConversionFactor);

        _talonLeader.configAllSettings(_talonConfig);
        _talonFollower.configAllSettings(_talonConfig);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.solenoidForwardChannel,
                RollerGripperConstants.solenoidReverseChannel);
    }

    public void periodic() {
        updateSDB();
    }

    public void updateSDB() {
        SmartDashboard.putBoolean("Auto Gripper Has Cone", hasCone());
    }

    public void setFolderState(FolderState state) {
        _doubleSolenoid.set(state.pneumaticState);
        _folderSM.setReference(state.desiredAngle, ControlType.kPosition);

        SmartDashboard.putString("Auto Gripper State", currentFolderState.toString());

        currentFolderState = state;
    }

    public void setRollersState(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, state.percentOutput);

    }

    public void stop() {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, 0);
        _folderSM.set(0);
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
