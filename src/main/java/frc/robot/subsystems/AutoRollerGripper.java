// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RollerGripperConstants;

public class AutoRollerGripper extends SubsystemBase {

    private TalonSRX _talonLeader, _talonFollower;
    private CANSparkMax _folderSM;
    private SparkMaxLimitSwitch _folderInLS, _folderOutLS;
    private DigitalInput _rollerLimitSwitch;

    public enum FolderState {
        IN(-0.3),
        OUT(0.3),
        OFF(0.0);

        private final double percentOutput;

        FolderState(double percentOutput) {
            this.percentOutput = percentOutput;
        }
    }

    public enum RollersState {
        INTAKE(0.70),
        EJECT(-0.70),
        OFF(0);

        private final double percentOutput;

        RollersState(double percentOutput) {
            this.percentOutput = percentOutput;
        }
    }

    public AutoRollerGripper() {
        _talonLeader = new TalonSRX(RollerGripperConstants.kTalonLeaderPort);
        _talonFollower = new TalonSRX(RollerGripperConstants.kTalonFollowerPort);
        _talonFollower.follow(_talonLeader);
        _talonFollower.setInverted(InvertType.OpposeMaster);

        _folderSM = new CANSparkMax(RollerGripperConstants.kSparkMaxFolderPort, MotorType.kBrushless);
        _folderInLS.enableLimitSwitch(true);
        _folderOutLS.enableLimitSwitch(true);
    }

    public void setFolderState(FolderState state) {
        _folderSM.set(state.percentOutput);
    }

    public void setRollersSpeed(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, state.percentOutput);
    }

    public boolean hasCone() {
        return _rollerLimitSwitch.get();
    }

    public CommandBase getIntakeCommand() {
        return new InstantCommand(() -> setFolderState(FolderState.OUT))
        .andThen(new StartEndCommand(
                () -> setRollersSpeed(RollersState.INTAKE),
                () -> setRollersSpeed(RollersState.OFF),
                this).until(this::hasCone));
    }

    public CommandBase getEjectCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                setRollersSpeed(RollersState.EJECT);
            }).andThen(new WaitCommand(0.05),
                new InstantCommand(() -> {
                setRollersSpeed(RollersState.OFF);
                setFolderState(FolderState.IN);
            }))
        );
    }
}
