package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private SwerveModule[] _modules;

    private PigeonIMU _pigeon;

    private SwerveDriveOdometry _odometry;
    private DoubleLogEntry m_logX, m_logY, m_logR;

    private static PIDController vision_xController;
    private static PIDController vision_yController;
    private static PIDController thetaController;

    public Drivetrain() {
        this._modules = new SwerveModule[] {
                new SwerveModule(DrivetrainConstants.TRModule),
                new SwerveModule(DrivetrainConstants.TLModule),
                new SwerveModule(DrivetrainConstants.BRModule),
                new SwerveModule(DrivetrainConstants.BLModule)
        };

        _pigeon = new PigeonIMU(DrivetrainConstants.pigeonId); // We need the talon; not anymore

        this._odometry = new SwerveDriveOdometry(DrivetrainConstants.kinematics, getRotation2d(),
                getSwerveModulePositions());

        DataLog log = DataLogManager.getLog();
        m_logX = new DoubleLogEntry(log, "/drivetrain/position/x");
        m_logY = new DoubleLogEntry(log, "/drivetrain/position/y");
        m_logR = new DoubleLogEntry(log, "/drivetrain/position/rotation");

        vision_xController = new PIDController(LimelightConstants.xGainsRetro.kP, LimelightConstants.xGainsRetro.kI,
                LimelightConstants.xGainsRetro.kD);
        vision_yController = new PIDController(LimelightConstants.yGainsRetro.kP, LimelightConstants.yGainsRetro.kI,
                LimelightConstants.yGainsRetro.kD);
        thetaController = new PIDController(LimelightConstants.thetaGains.kP,
                LimelightConstants.thetaGains.kI,
                LimelightConstants.thetaGains.kD);

        restartControllers();
    }

    public void setModulesAngle(double angle) {
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setAngle(state);
        }
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        fieldRelative = fieldRelative && this._pigeon.getState() == PigeonState.Ready;
        SmartDashboard.putBoolean("Field Relative", fieldRelative);

        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        var moduleStates = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

        setDesiredStates(moduleStates);
    }

    public void setDesiredStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                DrivetrainConstants.SwerveModuleConstants.freeSpeedMetersPerSecond);

        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        this._odometry.update(getRotation2d(), getSwerveModulePositions());

        // updateSDB();
        SmartDashboard.putNumber("pitch", this.getPitch());
    }

    public void updateTelemetry() {
        Pose2d pose = _odometry.getPoseMeters();
        m_logX.append(pose.getX());
        m_logY.append(pose.getY());
        m_logR.append(pose.getRotation().getDegrees());
    }

    public void disabledInit() {
        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].disable();
        }
    }

    @SuppressWarnings({ "unused" })
    private void updateSDB() {
        for (int i = 0; i < this._modules.length; i++) {
            SmartDashboard.putNumber("abs " + i, this._modules[i].getAbsAngle());
        }

        SmartDashboard.putNumber("rotation", getRotation2d().getRadians());
    }

    public Pose2d getPose() {
        return this._odometry.getPoseMeters();
    }

    public double getHeading() {
        return this._pigeon.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetYaw() {
        Pose2d pose = getPose();
        this.resetPose(new Pose2d(pose.getTranslation(), new Rotation2d()));
    }

    public void resetPose(Pose2d pose) {
        this._odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    public void calibrateSteering() {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.calibrateSteering();
        }
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[this._modules.length];

        for (int i = 0; i < swerveModulePositions.length; i++) {
            swerveModulePositions[i] = this._modules[i].getSwerveModulePosition();
        }

        return swerveModulePositions;
    }

    public void restartControllers() {
        vision_xController.reset();
        vision_yController.reset();
        thetaController.reset();

        vision_xController.setTolerance(LimelightConstants.xTol);
        vision_yController.setTolerance(LimelightConstants.yTol);
        thetaController.setTolerance(LimelightConstants.thetaTol);

        vision_xController.setSetpoint(0);
        vision_yController.setSetpoint(0);
        thetaController.setSetpoint(DrivetrainConstants.installAngle.getDegrees());
    }

    public void setVisionPIDsByInputs(double xp, double xi, double xd, double yp, double yi, double yd, double tp,
            double ti, double td) {
        vision_xController.setP(xp);
        vision_xController.setI(xi);
        vision_xController.setD(xd);

        vision_yController.setP(yp);
        vision_yController.setI(yi);
        vision_yController.setD(yd);

        thetaController.setP(tp);
        thetaController.setI(ti);
        thetaController.setD(td);
    }

    public void driveByVisionControllers(double Xangle, double Yangle) {
        double x = 0;
        double y = 0;
        double t = thetaController.calculate(this.getPose().getRotation().getDegrees());

        if (Math.abs(this.getPose().getRotation().getDegrees()
                - DrivetrainConstants.installAngle.getDegrees()) < LimelightConstants.spinToleranceDegrees) {
            // Don't move until we're somewhat aligned
            x = vision_xController.calculate(Xangle);
            y = vision_yController.calculate(Yangle);
        }

        this.drive(x, y, t, true);
    }

    public void setVisionPIDFromSDB() {
        this.setVisionPIDsByInputs(
                SmartDashboard.getNumber("xKP", 0), 0, SmartDashboard.getNumber("xKD", 0),
                SmartDashboard.getNumber("yKP", 0), 0, SmartDashboard.getNumber("yKD", 0),
                SmartDashboard.getNumber("tKP", 0), 0, SmartDashboard.getNumber("tKD", 0));

        vision_xController.setTolerance(SmartDashboard.getNumber("xKTol", 0));
        vision_yController.setTolerance(SmartDashboard.getNumber("yKTol", 0));
        thetaController.setTolerance(SmartDashboard.getNumber("tKTol", 0));
    }

    public void visionInitSDB() {
        SmartDashboard.putNumber("xKP", LimelightConstants.xGainsRetro.kP);
        SmartDashboard.putNumber("xKD", LimelightConstants.xGainsRetro.kD);
        SmartDashboard.putNumber("yKP", LimelightConstants.yGainsRetro.kP);
        SmartDashboard.putNumber("yKD", LimelightConstants.yGainsRetro.kD);
        SmartDashboard.putNumber("tKP", LimelightConstants.thetaGains.kP);
        SmartDashboard.putNumber("tKD", LimelightConstants.thetaGains.kD);

        SmartDashboard.putNumber("xKTol", LimelightConstants.xTol);
        SmartDashboard.putNumber("yKTol", LimelightConstants.yTol);
        SmartDashboard.putNumber("tKTol", LimelightConstants.thetaTol);

        SmartDashboard.putData("update vision SDB", new InstantCommand(() -> this.setVisionPIDFromSDB()));
    }

    public void setVisionRetroPID() {
        setVisionPIDsByInputs(
                LimelightConstants.xGainsRetro.kP, LimelightConstants.xGainsRetro.kI, LimelightConstants.xGainsRetro.kD,
                LimelightConstants.yGainsRetro.kP, LimelightConstants.yGainsRetro.kI, LimelightConstants.yGainsRetro.kD,
                LimelightConstants.thetaGains.kP, LimelightConstants.thetaGains.kI, LimelightConstants.thetaGains.kD);
    }

    public void setVisionAprilPID() {
        setVisionPIDsByInputs(
                LimelightConstants.xGainsApril.kP, LimelightConstants.xGainsApril.kI, LimelightConstants.xGainsApril.kD,
                LimelightConstants.yGainsApril.kP, LimelightConstants.yGainsApril.kI, LimelightConstants.yGainsApril.kD,
                LimelightConstants.thetaGains.kP, LimelightConstants.thetaGains.kI, LimelightConstants.thetaGains.kD);
    }

    public void spinAndDrive(double xSpeed, double ySpeed, Rotation2d targetRotation, boolean fieldRelative) {
        this.drive(xSpeed, ySpeed, thetaController.calculate(this.getPose().getRotation().getRadians()), fieldRelative);
    }

    public double getPitch() {
        return _pigeon.getPitch();
    }
}
