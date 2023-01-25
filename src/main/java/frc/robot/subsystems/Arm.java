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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.DynamicCommand;

public class Arm extends SubsystemBase {
    private static final int TICKS_PER_REVOLUTION = 4096;
    private TalonFX _leader;
    private TalonFX _follower;
    private ArmFeedforward _feedForward;
    private TalonFXConfiguration _leaderConfig = new TalonFXConfiguration();

    private ArmState _targetState;

    public static enum ArmState {
        COLLECT(ArmConstants.collectAngle),
        DRIVE(ArmConstants.driveAngle),
        MID_CONE(ArmConstants.midConeAngle),
        MID_CUBE(ArmConstants.midCubeAngle),
        LOW(ArmConstants.lowAngle);

        public final double stateAngle;

        private ArmState(double stateAngle) {
            this.stateAngle = stateAngle;
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

        _leader.configAllSettings(_leaderConfig);

        _follower.configAllSettings(new TalonFXConfiguration());
        _follower.follow(_leader);
        _follower.setInverted(InvertType.OpposeMaster);

        _leader.setSelectedSensorPosition(angleToTicks(getInitialState().stateAngle));
        _targetState = getInitialState();
    }

    private ArmState getInitialState() {
        if (_leader.isRevLimitSwitchClosed() == 1) {
            return ArmState.LOW;
        } else if (_leader.isFwdLimitSwitchClosed() == 1) {
            return ArmState.COLLECT;
        } else {
            return ArmState.DRIVE;
        }
    }

    public ArmState getTargetState() {
        return this._targetState;
    }

    public double getAngle() {
        return ticksToAngle(_leader.getSelectedSensorPosition());
    }

    private static double angleToTicks(double angle) {
        /*
         * Divides angle by 360 to get the percentage, then multiplies by ticks per
         * revolution constant to get angle in ticks,
         * then divides by gear ratio to get the actual angle the motor should move
         */
        return angle / 360 * TICKS_PER_REVOLUTION / ArmConstants.gearRatio;
    }

    private static double ticksToAngle(double ticks) {
        // Inverse of angleToTicks
        return ticks / angleToTicks(1);
    }

    private void useState(TrapezoidProfile.State state) {
        double feedForward = _feedForward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
        _leader.set(ControlMode.Position, angleToTicks(state.position), DemandType.ArbitraryFeedForward, feedForward);
        SmartDashboard.putNumber("State arm velocity", state.velocity);
        SmartDashboard.putNumber("State arm position", state.position);
    }

    private CommandBase generateSetStateCommand(ArmState requiredState) {
        TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.trapezoidConstraints,
                new TrapezoidProfile.State(requiredState.stateAngle, 0),
                new TrapezoidProfile.State(getAngle(), getVelocity()));

        return new TrapezoidProfileCommand(profile, this::useState, this)
                .alongWith(new InstantCommand(() -> _targetState = requiredState));
    }

    public CommandBase getSetStateCommand(ArmState requiredState) {
        return new DynamicCommand(() -> generateSetStateCommand(requiredState), this);
    }

    public double getVelocity() {
        return ticksToAngle(_leader.getSelectedSensorVelocity()) * 10;
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Current arm angle", getAngle());
        SmartDashboard.putString("Target arm state", getTargetState().toString());
        SmartDashboard.putNumber("Current arm velocity", getVelocity());
    }

    @Override
    public void periodic() {
        if (_leader.isFwdLimitSwitchClosed() == 1) {
            _leader.setSelectedSensorPosition(angleToTicks(ArmConstants.collectAngle));
        }
        updateSDB();
    }

    public void stop() {
        _leader.set(ControlMode.PercentOutput, 0);
    }
}