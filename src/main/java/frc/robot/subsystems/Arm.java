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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.DynamicCommand;

public class Arm extends SubsystemBase {
    private static final int TICKS_PER_REVOLUTION = 2048;
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

    private Constraints trapezoidConstraints = ArmConstants.trapezoidConstraints;
    private double startTime = Timer.getFPGATimestamp();
    private final DataLog log = DataLogManager.getLog();
    private final DoubleLogEntry time = new DoubleLogEntry(log, "/arm/calibrate/time");
    private final DoubleLogEntry angle = new DoubleLogEntry(log, "/arm/calibrate/angle");
    private final DoubleLogEntry velocity = new DoubleLogEntry(log, "/arm/calibrate/velocity");
    private final DoubleLogEntry stateAngle = new DoubleLogEntry(log, "/arm/calibrate/stateAngle");
    private final DoubleLogEntry stateVelocity = new DoubleLogEntry(log, "/arm/calibrate/stateVelocity");

    public void log(TrapezoidProfile.State state) {
        time.append(Timer.getFPGATimestamp() - startTime);
        angle.append(getAngle());
        velocity.append(getVelocity());
        stateAngle.append(state.position);
        stateVelocity.append(state.velocity);
    }

    private void loadFromSDB() {
        double movementRange = SmartDashboard.getNumber("movementRange", ArmConstants.movementRange);
        double movementTime = SmartDashboard.getNumber("movementTime", ArmConstants.movementTime);

        double maxVelocityDegreesPerSec = movementRange * 2 / movementTime;

        this.trapezoidConstraints = new Constraints(maxVelocityDegreesPerSec,
                maxVelocityDegreesPerSec / (movementTime / 2));

        this._feedForward = new ArmFeedforward(SmartDashboard.getNumber("staticGain", ArmConstants.staticGain),
                SmartDashboard.getNumber("gravityGain", ArmConstants.gravityGain),
                SmartDashboard.getNumber("velocityGain", ArmConstants.velocityGain),
                SmartDashboard.getNumber("accelerationGain", ArmConstants.accelerationGain));

        _leader.config_kP(0, SmartDashboard.getNumber("kP", ArmConstants.kP), 20);
        _leader.config_kI(0, SmartDashboard.getNumber("kI", 0.0), 20);
        _leader.config_kD(0, SmartDashboard.getNumber("kD", 0.0), 20);

        _leader.configPeakOutputForward(SmartDashboard.getNumber("kMaxOutput", ArmConstants.kMaxOutput), 20);
        _leader.configPeakOutputReverse(-SmartDashboard.getNumber("kMaxOutput", ArmConstants.kMaxOutput), 20);
    }

    public void setPercentSDB() {
        _leader.set(ControlMode.PercentOutput, SmartDashboard.getNumber("PercentOutput", 0.0));
    }

    public void setFeedFFSDB() {
        double feedForward = _feedForward.calculate(Math.toRadians(getAngle()), Math.toRadians(0)); // for kg
        // double feedForward = _feedForward.calculate(Math.toRadians(getAngle()),
        // Math.toRadians(0.00001)); // for ks

        // double feedForward = _feedForward.calculate(Math.toRadians(getAngle()),
        // Math.toRadians(0.00001))+0.3; // for kv

        // double feedForward = _feedForward.calculate(Math.toRadians(getAngle()),
        // Math.toRadians(50)); // for checking kv

        _leader.set(ControlMode.PercentOutput, feedForward);
    }

    private void initSDB() {
        SmartDashboard.putNumber("movementRange", ArmConstants.movementRange);
        SmartDashboard.putNumber("movementTime", ArmConstants.movementTime);
        SmartDashboard.putNumber("staticGain", ArmConstants.staticGain);
        SmartDashboard.putNumber("gravityGain", ArmConstants.gravityGain);
        SmartDashboard.putNumber("velocityGain", ArmConstants.velocityGain);
        SmartDashboard.putNumber("accelerationGain", ArmConstants.accelerationGain);
        SmartDashboard.putNumber("kP", ArmConstants.kP);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("kMaxOutput", ArmConstants.kMaxOutput);
        SmartDashboard.putData("COLLECT", getSetStateCommand(ArmState.COLLECT));
        SmartDashboard.putData("CONE", getSetStateCommand(ArmState.MID_CONE));
        SmartDashboard.putData("LOW", getSetStateCommand(ArmState.LOW));
        SmartDashboard.putData("DRIVE", getSetStateCommand(ArmState.DRIVE));

        SmartDashboard.putNumber("PercentOutput", 0.0);

        SmartDashboard.putData("loadFromSDB", new InstantCommand(() -> this.loadFromSDB()));
        SmartDashboard.putData("setPercentSDB", new InstantCommand(() -> this.setPercentSDB()));
        SmartDashboard.putData("setFeedFFSDB", new RunCommand(() -> this.setFeedFFSDB()));
        SmartDashboard.putData("stop", new InstantCommand(() -> this.stop()));
    }

    public Arm() {
        initSDB();
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
        _leaderConfig.neutralDeadband = 0.001;

        _leader.configAllSettings(_leaderConfig);
        _leader.setInverted(InvertType.InvertMotorOutput);

        _follower.configAllSettings(new TalonFXConfiguration());
        _follower.follow(_leader);
        _follower.setInverted(InvertType.OpposeMaster);

        _leader.setSelectedSensorPosition(angleToTicks(getInitialState().stateAngle));
        _targetState = getInitialState();
    }

    private ArmState getInitialState() {
        if (_leader.isRevLimitSwitchClosed() == 1) {
            return ArmState.COLLECT;
        } else if (_leader.isFwdLimitSwitchClosed() == 1) {
            return ArmState.LOW;
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
        log(state);
    }

    private CommandBase generateSetStateCommand(ArmState requiredState) {
        startTime = Timer.getFPGATimestamp();
        TrapezoidProfile profile = new TrapezoidProfile(trapezoidConstraints,
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
        SmartDashboard.putBoolean("fwd limit", _leader.isFwdLimitSwitchClosed() == 1);
        SmartDashboard.putBoolean("rev limit", _leader.isRevLimitSwitchClosed() == 1);
    }

    @Override
    public void periodic() {
        if (_leader.isRevLimitSwitchClosed() == 1) {
            _leader.setSelectedSensorPosition(angleToTicks(ArmConstants.collectAngle));
        }
        updateSDB();
    }

    public void stop() {
        _leader.set(ControlMode.PercentOutput, 0);
    }
}
