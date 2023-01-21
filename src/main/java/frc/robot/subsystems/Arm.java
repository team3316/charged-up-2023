package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    private static final int TICKS_PER_REVOLUTION = 4096;
    private TalonFX _leader;
    private TalonFX _follower;
    private ArmFeedforward _feedForward;
    private TalonFXConfiguration _leaderConfig = new TalonFXConfiguration();

    private ArmState _currentState;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle),
        DRIVE(ArmConstants.driveAngle),
        MID_CONE(ArmConstants.midConeAngle),
        MID_CUBE(ArmConstants.midCubeAngle),
        LOW(ArmConstants.lowAngle);

        public final double stateAngle;

        private ArmState(double StateAngle) {
            this.stateAngle = StateAngle;
        }
    }

    public Arm() {
        _leader = new TalonFX(ArmConstants.leaderCANID);
        _follower = new TalonFX(ArmConstants.followerCANID);

        _feedForward = new ArmFeedforward(ArmConstants.staticGain,
                ArmConstants.gravityGain,
                ArmConstants.velocityGain,
                ArmConstants.accelerationGain);

        _leaderConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        _leaderConfig.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        _leaderConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        _leaderConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        _leaderConfig.slot0.kP = ArmConstants.kP;
        _leaderConfig.peakOutputForward = ArmConstants.kMaxOutput;
        _leaderConfig.peakOutputReverse = -ArmConstants.kMaxOutput;

        _leader.setSelectedSensorPosition(0);
        _leader.configAllSettings(_leaderConfig);

        _follower.configAllSettings(new TalonFXConfiguration());
        _follower.follow(_leader);
        _follower.setInverted(InvertType.OpposeMaster);

        _leader.setSelectedSensorPosition(angleToTicks(getArmAngle()));
        
    }

    public double getArmAngle() {
        // TODO: need to get exact angles from Guy
        if (_leader.isRevLimitSwitchClosed() == 1) {
            return ArmConstants.lowAngle;
        } else if (_leader.isFwdLimitSwitchClosed() == 1) {
            return ArmConstants.collectAngle;
        } else {
            return ArmConstants.driveAngle;
        }
    }

    public ArmState getState() {
        return this._currentState;
    }

    public Command setStateCommand(ArmState requireState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.trapezoidConstraints,
                new TrapezoidProfile.State(getArmAngle(), 0),
                new TrapezoidProfile.State(requireState.stateAngle, 0));

        return new TrapezoidProfileCommand(profile, this::useState, this);
    }

    private void useState(TrapezoidProfile.State state) {

        double feedForward = _feedForward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));

        _leader.set(ControlMode.Position, angleToTicks(state.position), DemandType.ArbitraryFeedForward, feedForward);
    }

    private static double angleToTicks(double angle) {
        return angle / 360 * TICKS_PER_REVOLUTION / ArmConstants.gearRatio;
    }

    @Override
    public void periodic() {
        if (_leader.isFwdLimitSwitchClosed() == 1) {
            _leader.setSelectedSensorPosition(angleToTicks(ArmConstants.collectAngle));
        }
    }

}
