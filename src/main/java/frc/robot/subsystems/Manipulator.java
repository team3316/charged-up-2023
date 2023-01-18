// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {

    private ManipulatorState _currentState;
    private DigitalInput _gamePieceDetector;

    private DoubleSolenoid _leftSolenoid;
    private DoubleSolenoid _rightSolenoid;

    public static enum ManipulatorState {
        OPEN(ManipulatorConstants.leftSolenoidOpenState, ManipulatorConstants.rightSolenoidOpenState),
        CUBE_HOLD(ManipulatorConstants.leftSolenoidCubeClosedState, ManipulatorConstants.rightSolenoidCubeClosedState),
        CONE_HOLD(ManipulatorConstants.leftSolenoidConeClosedState, ManipulatorConstants.rightSolenoidConeClosedState);

        public final DoubleSolenoid.Value leftSolenoidState;
        public final DoubleSolenoid.Value rightSolenoidState;

        private ManipulatorState(DoubleSolenoid.Value LeftSolenoidState, DoubleSolenoid.Value RightSolenoidState) {
            this.leftSolenoidState = LeftSolenoidState;
            this.rightSolenoidState = RightSolenoidState;
        }
    }

    /** Creates a new Manipulator. */
    public Manipulator() {
        this._gamePieceDetector = new DigitalInput(ManipulatorConstants.sensorID);
        this._leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                ManipulatorConstants.leftSolenoidForwardChannel,
                ManipulatorConstants.leftSolenoidReverseChannel);
        this._rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                ManipulatorConstants.rightSolenoidForwardChannel,
                ManipulatorConstants.rightSolenoidReverseChannel);
    }

    public ManipulatorState getManipulatorState() {
        return this._currentState;
    }

    public void setManipulatorState(ManipulatorState requiredState) {
        if (this.getManipulatorState() == requiredState) {
            return;
        }

        this._leftSolenoid.set(requiredState.leftSolenoidState);
        this._rightSolenoid.set(requiredState.rightSolenoidState);

        SmartDashboard.putString("Manipulator State", requiredState.toString());
        this._currentState = requiredState;
    }

    public boolean isHoldingGamePiece() {
        return this._gamePieceDetector.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has GP", this.isHoldingGamePiece());
    }

    public CommandBase setManipulatorStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> {
            setManipulatorState(state);
        }, this);
    }
}
