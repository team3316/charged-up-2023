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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

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
    private RelativeEncoder _encoder;
    private SparkMaxPIDController _folderPID;

    private DoubleSolenoid _doubleSolenoid;

    private TalonSRXConfiguration _talonConfig = new TalonSRXConfiguration();

    private FolderState currentFolderState;

    /**
     * TODO: set all states from percentOutput to Position ControlMode
     * examples in:
     * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
     */

    public enum FolderState {
        IN(RollerGripperConstants.kFolderInValue, PneumaticFolderState.IN),
        OUT(RollerGripperConstants.kFolderOutValue, PneumaticFolderState.OUT),
        OFF(RollerGripperConstants.kFolderOffValue, PneumaticFolderState.OFF);

        private final double desiredAngle;
        private final PneumaticFolderState pneumaticState;

        FolderState(double desiredAngle, PneumaticFolderState pneumaticState) {
            this.desiredAngle = desiredAngle;
            this.pneumaticState = pneumaticState;
        }
    }

    public enum PneumaticFolderState {
        OUT(RollerGripperConstants.kStateWhenFoldedOut),
        IN(RollerGripperConstants.kStateWhenFoldedIn),
        OFF(RollerGripperConstants.kStateWhenOff);

        private DoubleSolenoid.Value solenoidValue;

        private PneumaticFolderState(DoubleSolenoid.Value value) {
            this.solenoidValue = value;
        }
    }

    public enum RollersState {
        INTAKE(RollerGripperConstants.kRollerIntakeValue),
        EJECT(RollerGripperConstants.kRollerEjectValue),
        OFF(RollerGripperConstants.kRollerOffValue);

        private final double desiredAngle;

        RollersState(double desiredAngle) {
            this.desiredAngle = desiredAngle;
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
        _encoder = _folderSM.getEncoder();
        _folderPID = _folderSM.getPIDController();

        _talonLeader.configAllSettings(_talonConfig);
        _talonFollower.configAllSettings(_talonConfig);

        _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RollerGripperConstants.kSolenoidForwardChannel,
                RollerGripperConstants.kSolenoidReverseChannel);

        _encoder.setPosition(0);

        _folderSM.restoreFactoryDefaults();
        _folderPID.setP(RollerGripperConstants.folderGains.kP);
        _folderPID.setI(RollerGripperConstants.folderGains.kI);
        _folderPID.setD(RollerGripperConstants.folderGains.kD);

    }

    public void periodic() {
        if (_encoder.getPosition() <= RollerGripperConstants.kMaxFolderIn ||
                _encoder.getPosition() >= RollerGripperConstants.kMaxFolderOut)
            setFolderState(FolderState.OFF);

        SmartDashboard.putString("current folder state", currentFolderState.toString());
        SmartDashboard.putBoolean("hasCone", hasCone());
    }

    public void setFolderState(FolderState state) {
        _folderSM.set(state.desiredAngle);
        _doubleSolenoid.set(state.pneumaticState.solenoidValue);

        currentFolderState = state;
    }

    public void setRollersState(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.Position, state.desiredAngle);
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
