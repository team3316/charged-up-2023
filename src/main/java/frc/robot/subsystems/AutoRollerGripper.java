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

    private DBugSparkMax _topSparkMax, _bottomSparkMax;
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
        INTAKE(RollerGripperConstants.rollerTopIntakePercent, RollerGripperConstants.rollerBottomIntakePercent),
        EJECT(RollerGripperConstants.rollerEjectPercent, RollerGripperConstants.rollerEjectPercent),
        OFF(RollerGripperConstants.rollerOffPercent, RollerGripperConstants.rollerOffPercent);

        private final double topPrecent;
        private final double bottomPrecent;

        RollersState(double topPrecent, double BottomPrecent) {
            this.topPrecent = topPrecent;
            this.bottomPrecent = BottomPrecent;
        }
    }

    public AutoRollerGripper() {
        _topSparkMax = DBugSparkMax.create(RollerGripperConstants.sparkMaxTopPort);
        _bottomSparkMax = DBugSparkMax.create(RollerGripperConstants.sparkMaxBottomPort);
        _bottomSparkMax.setInverted(true);

        _topSparkMax.setSmartCurrentLimit(RollerGripperConstants.currentLimitAmp);
        _bottomSparkMax.setSmartCurrentLimit(RollerGripperConstants.currentLimitAmp);

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
        _topSparkMax.set(state.topPrecent);
        _bottomSparkMax.set(state.bottomPrecent);

    }

    public void stop() {
        setRollersState(RollersState.OFF);
        _doubleSolenoid.set(Value.kOff);
    }

    public boolean hasCone() {
        return !_rollerLimitSwitch.get();
    }

    public CommandBase getIntakeCommand() {
        return Commands.sequence(
                new InstantCommand(() -> this.setRollersState(RollersState.INTAKE)),
                new WaitUntilCommand(this::hasCone),
                new WaitCommand(RollerGripperConstants.intakeSleepDurationSeconds),
                new InstantCommand(() -> this.setRollersState(RollersState.OFF)));
    }

    public CommandBase getEjectCommand() {
        return new InstantCommand(() -> {
            this.setRollersState(RollersState.EJECT);
        }).andThen(new WaitCommand(RollerGripperConstants.ejectSleepDurationSeconds),
                new InstantCommand(() -> {
                    this.setRollersState(RollersState.OFF);
                }));
    }

    public CommandBase getFoldCommand(FolderState fState) {
        return new InstantCommand(() -> {
            this.setFolderState(fState);
        });
    }
}
