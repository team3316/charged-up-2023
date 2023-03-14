// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.RollerGripperConstants;
import frc.robot.motors.DBugSparkMax;

public class AutoRollerGripper extends SubsystemBase {

    private DBugSparkMax _rightSparkMax, _leftSparkMax;
    private DigitalInput _rollerLimitSwitch;

    private DoubleSolenoid _doubleSolenoid;

    private FolderState currentFolderState = FolderState.IN;

    public enum FolderState {
        IN(RollerGripperConstants.stateWhenFoldedIn),
        OUT(RollerGripperConstants.stateWhenFoldedOut);

        private final DoubleSolenoid.Value pneumaticState;

        FolderState(DoubleSolenoid.Value pneumaticState) {
            this.pneumaticState = pneumaticState;
        }
    }

    public enum RollersState {
        INTAKE(RollerGripperConstants.rollerRightIntakePercent, RollerGripperConstants.rollerLeftIntakePercent),
        EJECT(RollerGripperConstants.rollerEjectPercent, RollerGripperConstants.rollerEjectPercent),
        OFF(RollerGripperConstants.rollerOffPercent, RollerGripperConstants.rollerOffPercent);

        private final double _rightPrecent;
        private final double _leftPrecent;

        RollersState(double rightPrecent, double leftPrecent) {
            this._rightPrecent = rightPrecent;
            this._leftPrecent = leftPrecent;
        }
    }

    public AutoRollerGripper() {
        _rightSparkMax = DBugSparkMax.create(RollerGripperConstants.sparkMaxRightPort);
        _leftSparkMax = DBugSparkMax.create(RollerGripperConstants.sparkMaxLeftPort);
        _leftSparkMax.setInverted(true);

        _rightSparkMax.setSmartCurrentLimit(RollerGripperConstants.currentLimitAmp);
        _leftSparkMax.setSmartCurrentLimit(RollerGripperConstants.currentLimitAmp);

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

        SmartDashboard.putString("Auto Gripper State", currentFolderState.toString());

        currentFolderState = state;
    }

    public void setRollersState(RollersState state) {
        _rightSparkMax.set(state._rightPrecent);
        _leftSparkMax.set(state._leftPrecent);

    }

    public void stop() {
        setRollersState(RollersState.OFF);
        _doubleSolenoid.set(Value.kOff);
    }

    public boolean hasCone() {
        return !_rollerLimitSwitch.get(); // limitSwitch is NC
    }

    public CommandBase getIntakeCommand() {
        CommandBase intakeSequence = Commands.sequence(
                new InstantCommand(() -> this.setRollersState(RollersState.INTAKE)),
                new WaitUntilCommand(this::hasCone),
                new InstantCommand(() -> this.setFolderState(FolderState.IN)),
                new WaitCommand(RollerGripperConstants.intakeSleepDurationSeconds),
                new InstantCommand(() -> this.setRollersState(RollersState.OFF)));
        intakeSequence.addRequirements(this);
        return intakeSequence;
    }

    public CommandBase getEjectCommand() {
        CommandBase ejectSequence = Commands.sequence(
                new InstantCommand(() -> this.setFolderState(FolderState.OUT)),
                new InstantCommand(() -> this.setRollersState(RollersState.EJECT)),
                new WaitCommand(RollerGripperConstants.ejectSleepDurationSeconds),
                new InstantCommand(() -> this.setRollersState(RollersState.OFF)),
                new InstantCommand(() -> this.setFolderState(FolderState.IN)));
        ejectSequence.addRequirements(this);
        return ejectSequence;
    }
}
