// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RollerGripperConstants;

public class AutoRollerGripper extends SubsystemBase {

    private DigitalInput _rollerGripperLS;
    private TalonSRX _talonLeader, _talonFollower;
    private CANSparkMax _folderSM;
    private SparkMaxLimitSwitch _folderInLS, _folderOutLS;

    public enum StateList {
        FOLDS_OUT,
        COLLECTING,
        RELEASE,
        FOLDS_IN
    }

    public enum RollersState {
        INTAKE(0.70),
        EJECT(-0.70),
        OFF(0);

        private final double speed;

        RollersState(double speed) {
            this.speed = speed;
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

    void folds(RollersState state) {
        _folderSM.set(state.speed);
    }

    public void setRollersSpeed(RollersState state) {
        _talonLeader.set(TalonSRXControlMode.PercentOutput, state.speed);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
