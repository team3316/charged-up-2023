package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.motors.DBugSparkMax;

/**
 * SwerveModule
 */
public class SwerveModule {

    private DBugSparkMax _driveMotor;
    private DBugSparkMax _steerMotor;

    private CANCoder _absEncoder;

    private SwerveModuleState _targetState;

    public SwerveModule(SwerveModuleConstants constants) {
        this._absEncoder = createCANCoder(constants.canCoderId, constants.cancoderZeroAngle);

        this._driveMotor = DBugSparkMax.create(
                constants.idDrive,
                constants.driveGains,
                SwerveModuleConstants.drivePositionConversionFactor,
                SwerveModuleConstants.driveVelocityConversionFactor,
                0);

        this._steerMotor = DBugSparkMax.create(
                constants.idSteering,
                constants.steeringGains,
                SwerveModuleConstants.steeringPositionConversionFactor,
                SwerveModuleConstants.steeringVelocityConversionFactor,
                _absEncoder.getAbsolutePosition());

        this._driveMotor.setAverageDepth(4);
        this._driveMotor.setMeasurementPeriod(8);

        this._targetState = getState();
    }

    private static CANCoder createCANCoder(int id, double zeroAngle) {

        CANCoder canCoder = new CANCoder(id);
        // Always set CANCoder relative encoder to 0 on boot
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        // Configure the offset angle of the magnet
        canCoder.configMagnetOffset(360 - zeroAngle, 30);

        return canCoder;
    }

    public void calibrateSteering() {
        double currentAngle = this._steerMotor.getPosition();
        double angleDiff = (getAbsAngle() - currentAngle) % 360;

        double targetAngle = currentAngle + angleDiff;
        if (angleDiff <= -180)
            targetAngle += 360;

        else if (angleDiff >= 180)
            targetAngle -= 360;

        if (Math.abs(targetAngle - currentAngle) > 2) // avoid sending unnecessary CAN packets
            this._steerMotor.setPosition(targetAngle);
    }

    public void stop() {
        _driveMotor.set(0);
        _steerMotor.set(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this._driveMotor.getVelocity(),
                new Rotation2d().rotateBy(Rotation2d.fromDegrees(this._steerMotor.getPosition())));
    }

    public SwerveModuleState getTargetState() {
        return _targetState;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, this._steerMotor.getPosition());

        if (state.speedMetersPerSecond != 0) // Avoid steering in place
            this._steerMotor.setReference(state.angle.getDegrees(), ControlType.kPosition);

        if (state.speedMetersPerSecond == 0)
            this.stop();
        else
            this._driveMotor.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        _targetState = desiredState;

    }

    public void setAngle(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        this._steerMotor.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        // desired angle diff in [-360, +360]
        double _angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;

        double targetAngle = currentAngle + _angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Q1 undershot. We expect a CW turn.
        if (_angleDiff <= -270)
            targetAngle += 360;

        // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
        // Q3. We expect a CW turn to Q1 & reverse direction.
        else if (-90 > _angleDiff && _angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        }

        // Q2. We expect a CCW turn to Q4 & reverse direction.
        // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
        else if (90 < _angleDiff && _angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        }

        // Q4 overshot. We expect a CCW turn.
        else if (_angleDiff >= 270)
            targetAngle -= 360;

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public double getAbsAngle() {
        return _absEncoder.getAbsolutePosition();
    }

    public void disable() {
        this.stop();
        this._steerMotor.set(0);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(_driveMotor.getPosition(), Rotation2d.fromDegrees(getAbsAngle()));
    }
}