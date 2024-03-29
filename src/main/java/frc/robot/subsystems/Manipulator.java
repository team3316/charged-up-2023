// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
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
    private LinearFilter _gamePieceDetectorFilter = LinearFilter.movingAverage(10);
    private Hysteresis _hysteresis = new Hysteresis(0, 0);
    private Hysteresis _funneling = new Hysteresis(IRSensorState.Funneling._bottomThreshold,
            IRSensorState.Funneling._hysteresis);

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

    private double getGamePieceDetectorValue() {
        return this._gamePieceDetectorFilter.calculate(this._gamePieceDetector.getValue() / ADC_RESOLUTION);
    }

    public boolean isHoldingGamePiece() {
        return _hysteresis.update(getGamePieceDetectorValue());
    }

    public boolean isFunnelingGamePiece() {
        return _funneling.update(getGamePieceDetectorValue());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has GP", this.isHoldingGamePiece());
        SmartDashboard.putBoolean("Funneling GP", this.isFunnelingGamePiece());
        SmartDashboard.putNumber("IR sensor value", getGamePieceDetectorValue());
    }

    public CommandBase setManipulatorStateCommand(ManipulatorState state) {
        return new InstantCommand(() -> {
            setManipulatorState(state);
        });
    }

    public static enum IRSensorState {
        Funneling(ManipulatorConstants.FunnelingDetectorThreshold, ManipulatorConstants.FunnelingDetectorHysteresis),
        CUBE(ManipulatorConstants.CUBEDetectorThreshold, ManipulatorConstants.CUBEDetectorHysteresis),
        CONE(ManipulatorConstants.CONEDetectorThreshold, ManipulatorConstants.CONEDetectorHysteresis);

        public final double _bottomThreshold;
        public final double _hysteresis;

        private IRSensorState(double bottomThreshold, double hysteresis) {
            _bottomThreshold = bottomThreshold;
            _hysteresis = hysteresis;
        }
    }

    public void setIRSensorState(IRSensorState wantedState) {
        _hysteresis = new Hysteresis(wantedState._bottomThreshold, wantedState._hysteresis);
    }
}
