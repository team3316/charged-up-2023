// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder;

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

public class AutoRollerGripper extends SubsystemBase {

    private TalonSRX _talonLeader, _talonFollower;
    private DigitalInput _rollerLimitSwitch;

    private SparkMaxLimitSwitch _folderInLS, _folderOutLS;
    private CANSparkMax _folderSM;
    private SparkMaxRelativeEncoder _encoder;

    private DoubleSolenoid _doubleSolenoid;

    private TalonSRXConfiguration _talonConfig = new TalonSRXConfiguration();

    /**
     * TODO: set all states from percentOutput to Position ControlMode
     * examples in:
     */

    public enum FolderState {
        IN(RollerGripperConstants.kFolderInValue, PneumaticFolderState.IN),
        OUT(RollerGripperConstants.kFolderOutValue, PneumaticFolderState.OUT),
        OFF(RollerGripperConstants.kFolderOffValue, PneumaticFolderState.OFF);

        private final double percentOutput;
        private final PneumaticFolderState pneumaticState;

        FolderState(double percentOutput, PneumaticFolderState pneumaticState) {
            this.percentOutput = percentOutput;
            this.pneumaticState = pneumaticState;
        }
    }

    public enum PneumaticFolderState {
        OUT(RollerGripperConstants.kStateWhenFoldedOut),
        IN(RollerGripperConstants.kStateWhenFoldedIn),
        OFF(null);

        private DoubleSolenoid.Value solenoidValue;

        private PneumaticFolderState(DoubleSolenoid.Value value) {
            this.solenoidValue = value;
        }
    }

    public enum RollersState {
        INTAKE(RollerGripperConstants.kRollerIntakeValue),
        EJECT(RollerGripperConstants.kRollerEjectValue),
        OFF(RollerGripperConstants.kRollerOffValue);

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

        _talonLeader.configAllSettings(_talonConfig);
        _talonFollower.configAllSettings(_talonConfig);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.kSolenoidForwardChannel,
                RollerGripperConstants.kSolenoidReverseChannel);

        _encoder.setPosition(0);

    }

    public void periodic() {
        SmartDashboard.putBoolean("hasCone", hasCone());

        if (_encoder.getPosition() <= RollerGripperConstants.kMaxFolderIn ||
                _encoder.getPosition() >= RollerGripperConstants.kMaxFolderOut)
            setFolderState(FolderState.OFF);
    }

    // TODO: merge the setFolderStates for double actions
    public void setFolderState(FolderState state) {
        _folderSM.set(state.percentOutput);
    }

    public void setFolderState(PneumaticFolderState pneumaticState) {
        _doubleSolenoid.set(pneumaticState.solenoidValue);
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
                        () -> setRollersState(RollersState.INTAKE),
                        () -> {
                            new WaitCommand(RollerGripperConstants.kGrippingSleepDuration).andThen(
                                    new InstantCommand(() -> setRollersState(RollersState.OFF)));
                        }).until(this::hasCone));
    }

    public CommandBase getEjectCommand() {
        return new InstantCommand(() -> {
            setRollersState(RollersState.EJECT);
        }).andThen(new WaitCommand(RollerGripperConstants.kFoldingSleepDuration),
                new InstantCommand(() -> {
                    setRollersState(RollersState.OFF);
                    setFolderState(FolderState.IN);
                }));
    }
}
