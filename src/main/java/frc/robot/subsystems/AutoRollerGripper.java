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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.RollerGripperConstants;
import frc.robot.motors.DBugSparkMax;

public class AutoRollerGripper extends SubsystemBase {

    private DBugSparkMax _leader, _follower;
    private DigitalInput _rollerLimitSwitch;

    private DoubleSolenoid _doubleSolenoid;

    private FolderState currentFolderState;

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
        _leader = DBugSparkMax.create(RollerGripperConstants.sparkMaxLeaderPort);
        _follower = DBugSparkMax.create(RollerGripperConstants.sparkMaxFollowerPort);
        _follower.follow(_leader, true);

        _leader.setSmartCurrentLimit(20);
        _follower.setSmartCurrentLimit(20);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.solenoidForwardChannel,
                RollerGripperConstants.solenoidReverseChannel);

        _rollerLimitSwitch = new DigitalInput(RollerGripperConstants.rollerLimitSwitchPort);

        SmartDashboard.putNumber("intake precent", SmartDashboard.getNumber("intake precent", 0));
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
        _leader.set(state.percentOutput);

    }

    public void stop() {
        _leader.set(0);
        _doubleSolenoid.set(Value.kOff);
    }

    public boolean hasCone() {
        return !_rollerLimitSwitch.get();
    }

    public CommandBase getIntakeCommand() {
        // return new StartEndCommand(
        // () -> {
        // setRollersState(RollersState.INTAKE);
        // },
        // () -> {
        // new WaitCommand(RollerGripperConstants.intakeSleepDurationSeconds).andThen(
        // new InstantCommand(() -> setRollersState(RollersState.OFF)));
        // }, this).until(this::hasCone);
        return Commands.sequence(
                new InstantCommand(() -> _leader.set(SmartDashboard.getNumber("intake precent", 0))),
                new WaitUntilCommand(this::hasCone),
                new WaitCommand(RollerGripperConstants.intakeSleepDurationSeconds),
                new InstantCommand(() -> setRollersState(RollersState.OFF)));
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
