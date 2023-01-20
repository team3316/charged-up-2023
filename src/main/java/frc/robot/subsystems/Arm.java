package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {

    private TalonFX _leader;
    private TalonFX _follower;
    private ArmFeedforward _feedforward;
    private TalonFXConfiguration _leaderConfig = new TalonFXConfiguration();

    private ArmState _currnetState;

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

        _feedforward = new ArmFeedforward(ArmConstants.staticGain,
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

        _currnetState = ArmState.COLLECT;
    }

    public double getArmAngle() {
        // need to get exact angles from Guy
        if (_leaderConfig.forwardSoftLimitEnable) {
            return ArmConstants.lowAngle;
        } else if (_leaderConfig.reverseSoftLimitEnable) {
            return ArmConstants.collectAngle;
        } else {
            return ArmConstants.driveAngle;
        }
    }

    public ArmState getState() {
        return this._currnetState;
    }

}
