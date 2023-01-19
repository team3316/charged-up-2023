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
        IN(RollerGripperConstants.inAngle, PneumaticFolderState.IN),
        OUT(RollerGripperConstants.outAngle, PneumaticFolderState.OUT),
        OFF(0, PneumaticFolderState.OFF);

        private final double desiredAngle;
        private final PneumaticFolderState pneumaticState;

        FolderState(double desiredAngle, PneumaticFolderState pneumaticState) {
            this.desiredAngle = desiredAngle;
            this.pneumaticState = pneumaticState;
        }
    }

    public enum PneumaticFolderState {
        OUT(RollerGripperConstants.stateWhenFoldedOut),
        IN(RollerGripperConstants.stateWhenFoldedIn),
        OFF(RollerGripperConstants.stateWhenOff);

        private DoubleSolenoid.Value solenoidValue;

        private PneumaticFolderState(DoubleSolenoid.Value value) {
            this.solenoidValue = value;
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

        _folderSM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);
        _folderSM.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);

        _folderSM.getEncoder().setPositionConversionFactor(RollerGripperConstants.positionConversionFactor);

        _talonLeader.configAllSettings(_talonConfig);
        _talonFollower.configAllSettings(_talonConfig);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.solenoidForwardChannel,
                RollerGripperConstants.solenoidReverseChannel);

        _folderSM.setupPIDF(RollerGripperConstants.folderGains);

    }

    public void periodic() {
        // updateSDB();
    }

    public void updateSDB() {
        SmartDashboard.putBoolean("hasCone", hasCone());
    }

    public void setFolderState(FolderState state) {
        if (state == FolderState.OFF) {
            _folderSM.set(0);
        } else {
            _doubleSolenoid.set(state.pneumaticState.solenoidValue);
            _folderSM.setReference(state.desiredAngle, ControlType.kPosition);
        }

        SmartDashboard.putString("current folder state", currentFolderState.toString());

        currentFolderState = state;
    }

    public void setRollersState(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, state.percentOutput);

    }

    public boolean hasCone() {
        return _rollerLimitSwitch.get();
    }

    public CommandBase getIntakeCommand() {
        return new InstantCommand(() -> setFolderState(FolderState.OUT))
                .andThen(new StartEndCommand(
                        () -> {
                            setRollersState(RollersState.INTAKE);
                        },
                        () -> {
                            new WaitCommand(RollerGripperConstants.grippingSleepDurationSeconds).andThen(
                                    new InstantCommand(() -> setRollersState(RollersState.OFF)));
                        }).until(this::hasCone));
    }

    public CommandBase getEjectCommand() {
        return new InstantCommand(() -> {
            setRollersState(RollersState.EJECT);
        }).andThen(new WaitCommand(RollerGripperConstants.foldingSleepDurationSeconds),
                new InstantCommand(() -> {
                    setRollersState(RollersState.OFF);
                    setFolderState(FolderState.IN);
                }));
    }
}
