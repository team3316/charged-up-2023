// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.utils.Hysteresis;

public class Manipulator extends SubsystemBase {

    private static final double ADC_RESOLUTION = 4096; // 12 bits
    private ManipulatorState _currentState;
    private AnalogInput _gamePieceDetector;
    private Hysteresis _hysteresis = new Hysteresis(ManipulatorConstants.GPDetectorThreshold,
            ManipulatorConstants.GPDetectorHysteresis);

    private DoubleSolenoid _solenoid;

    public static enum ManipulatorState {
        OPEN(ManipulatorConstants.solenoidOpenState),
        HOLD(ManipulatorConstants.solenoidClosedState);

        public final DoubleSolenoid.Value solenoidState;

        private ManipulatorState(DoubleSolenoid.Value solenoidState) {
            this.solenoidState = solenoidState;
        }
    }

    /** Creates a new Manipulator. */
    public Manipulator() {
        this._gamePieceDetector = new AnalogInput(ManipulatorConstants.sensorID);
        this._solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                ManipulatorConstants.solenoidForwardChannel,
                ManipulatorConstants.solenoidReverseChannel);
    }

    public ManipulatorState getManipulatorState() {
        return this._currentState;
    }

    public void setManipulatorState(ManipulatorState requiredState) {
        if (this.getManipulatorState() == requiredState) {
            return;
        }

        this._solenoid.set(requiredState.solenoidState);

        SmartDashboard.putString("Manipulator State", requiredState.toString());
        this._currentState = requiredState;
    }

    public boolean isHoldingGamePiece() {
        return _hysteresis.update(this._gamePieceDetector.getValue() / ADC_RESOLUTION);
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
