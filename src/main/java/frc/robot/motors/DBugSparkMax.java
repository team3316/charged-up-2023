package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import frc.robot.constants.DrivetrainConstants;

public class DBugSparkMax extends CANSparkMax {
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;
    private boolean _isCalibrated = false;

    public DBugSparkMax(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);

        this._encoder = this.getEncoder();

        this._pidController = this.getPIDController();
    }

    public void setReference(
            double value,
            ControlType ctrl,
            int pidSlot,
            double arbFeedforward,
            ArbFFUnits arbFFUnits) {
        this._pidController.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

    public void setReference(double value, ControlType ctrl) {
        this._pidController.setReference(value, ctrl);
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this._encoder.setPositionConversionFactor(positionFactor);
        this._encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setupPIDF(PIDFGains gains) {
        this._pidController.setP(gains.kP);
        this._pidController.setI(gains.kI);
        this._pidController.setD(gains.kD);
        this._pidController.setFF(gains.kF);
        this._pidController.setIZone(gains.iZone);
        this._pidController.setOutputRange(-gains.outputRange, gains.outputRange);
    }

    public void setPosition(double value) {
        this._encoder.setPosition(value);
    }

    public double getVelocity() {
        return this._encoder.getVelocity();

    }

    public double getPosition() {
        return this._encoder.getPosition();
    }

    public static DBugSparkMax create(
            int id,
            PIDFGains gains,
            double positionFactor,
            double velocityFactor,
            double position) {
        DBugSparkMax sparkMax = new DBugSparkMax(id);
        sparkMax.restoreFactoryDefaults();
        sparkMax.setCANTimeout(50);
        sparkMax.setupPIDF(gains);
        sparkMax.setConversionFactors(positionFactor, velocityFactor);
        sparkMax.setSmartCurrentLimit(40);
        sparkMax.enableVoltageCompensation(12);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setOpenLoopRampRate(0.01);
        sparkMax.setClosedLoopRampRate(0.01);
        sparkMax.setPosition(position);
        return sparkMax;
    }

    public static DBugSparkMax create(int id) {
        return create(id, new PIDFGains(0), 1, 1, 0);
    }

    public void testAndFixDriveController() {
        if (_isCalibrated) {
            return;
        }
        _isCalibrated = true;

        if (_pidController.getP() != DrivetrainConstants.SwerveModuleConstants.driveKp) {
            _pidController.setP(DrivetrainConstants.SwerveModuleConstants.driveKp);
            _isCalibrated = false;
        }

        if (_pidController.getFF() != DrivetrainConstants.SwerveModuleConstants.driveKf) {
            _pidController.setFF(DrivetrainConstants.SwerveModuleConstants.driveKf);
            _isCalibrated = false;
        }

        if (_encoder
                .getVelocityConversionFactor() != DrivetrainConstants.SwerveModuleConstants.driveVelocityConversionFactor) {
            _encoder.setVelocityConversionFactor(
                    DrivetrainConstants.SwerveModuleConstants.driveVelocityConversionFactor);
            _isCalibrated = false;
        }

        if (_encoder
                .getPositionConversionFactor() != DrivetrainConstants.SwerveModuleConstants.drivePositionConversionFactor) {
            _encoder.setPositionConversionFactor(
                    DrivetrainConstants.SwerveModuleConstants.drivePositionConversionFactor);
            _isCalibrated = false;
        }
    }

    public void testAndFixSteerController() {
        if (_isCalibrated) {
            return;
        }
        _isCalibrated = true;

        if (_pidController.getP() != DrivetrainConstants.SwerveModuleConstants.steeringKp) {
            _pidController.setP(DrivetrainConstants.SwerveModuleConstants.steeringKp);
            _isCalibrated = false;
        }

        if (_encoder
                .getVelocityConversionFactor() != DrivetrainConstants.SwerveModuleConstants.steeringVelocityConversionFactor) {
            _encoder.setVelocityConversionFactor(
                    DrivetrainConstants.SwerveModuleConstants.steeringVelocityConversionFactor);
            _isCalibrated = false;
        }

        if (_encoder
                .getPositionConversionFactor() != DrivetrainConstants.SwerveModuleConstants.steeringPositionConversionFactor) {
            _encoder.setPositionConversionFactor(
                    DrivetrainConstants.SwerveModuleConstants.steeringPositionConversionFactor);
            _isCalibrated = false;
        }
    }

    public void setMeasurementPeriod(int period_ms) {
        this._encoder.setMeasurementPeriod(period_ms);
    }

    public void setAverageDepth(int depth) {
        this._encoder.setAverageDepth(depth);
    }

    public int getMeasurementPeriod() {
        return this._encoder.getMeasurementPeriod();
    }
}
