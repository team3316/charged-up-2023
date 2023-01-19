// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FunnelConstants;

public class Funnel extends SubsystemBase {

    private CANSparkMax _rollers;
    private FunnelState _currentState;

    public enum FunnelState {
        COLLECT(FunnelConstants.collectState, FunnelConstants.collectSpeed),
        INSTALL(FunnelConstants.collectState, FunnelConstants.installSpeed),
        CLOSED(DoubleSolenoid.Value.kReverse, FunnelConstants.closedSpeed);

        public final DoubleSolenoid.Value solenoidState;
        public final double rollerSpeed;

        private FunnelState(Value solenoidState, double rollerSpeed) {
            this.solenoidState = solenoidState;
            this.rollerSpeed = rollerSpeed;
        }
    };

    /** Creates a new Funnel. */
    private DoubleSolenoid _funnelSolenoid;

    public Funnel() {
        this._funnelSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, FunnelConstants.solenoidForwardPort,
                FunnelConstants.solenoidReversePort);
        this._rollers = new CANSparkMax(FunnelConstants.sparkMaxPort, MotorType.kBrushless);

    }

    public FunnelState getFunnelState() {
        return _currentState;

    }

    public void setFunnelState(FunnelState state) {
        if (state == getFunnelState()) {
            return;
        }

        _funnelSolenoid.set(state.solenoidState);
        _rollers.set(state.rollerSpeed);

        _currentState = state;
    }

    @Override
    public void periodic() {
        // updateSDB();
    }

    @SuppressWarnings({ "unused" })
    private void updateSDB() {
        SmartDashboard.putString("funnel state", this._currentState.toString());
        SmartDashboard.putNumber("funnel roller speed", this._rollers.get());

    }

    public CommandBase setFunnelStateCommand(FunnelState state) {
        return new InstantCommand(() -> setFunnelState(state), this);
    }
}
