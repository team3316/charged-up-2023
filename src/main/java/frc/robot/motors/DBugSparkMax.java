package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import frc.robot.utils.Within;

public class DBugSparkMax extends CANSparkMax {
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;

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

        if (!sparkMax.verify(gains, positionFactor, velocityFactor)) {
            throw new IllegalStateException("SparkMax Failed to Initialize");
        }
        return sparkMax;
    }

    public static DBugSparkMax create(int id) {
        return create(id, new PIDFGains(0), 1, 1, 0);
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

    public boolean verify(PIDFGains gains, double positionFactor, double velocityFactor) {
        if (!Within.range(gains.kP, this._pidController.getP(), 0.001))
            return false;
        if (!Within.range(gains.kI, this._pidController.getI(), 0.001))
            return false;
        if (!Within.range(gains.kD, this._pidController.getD(), 0.001))
            return false;
        if (!Within.range(gains.kF, this._pidController.getFF(), 0.001))
            return false;
        if (!Within.range(positionFactor, this._encoder.getPositionConversionFactor(), 0.001))
            return false;
        if (!Within.range(velocityFactor, this._encoder.getVelocityConversionFactor(), 0.001))
            return false;
        return true;
    }
}
