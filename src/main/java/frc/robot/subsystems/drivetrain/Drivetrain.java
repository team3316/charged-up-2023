package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
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

    private DoubleLogEntry m_logX, m_logY, m_logR, m_logwV, m_logV;

    private Double posErrorX, posErrorY, rotationError;
    private PathPlannerState state;

    private static PIDController vision_xController;
    private static PIDController vision_yController;
    private static PIDController vision_thetaController;

    public Drivetrain(DataLog log) {
        this._modules = new SwerveModule[] {
                new SwerveModule(DrivetrainConstants.TRModule),
                new SwerveModule(DrivetrainConstants.TLModule),
                new SwerveModule(DrivetrainConstants.BRModule),
                new SwerveModule(DrivetrainConstants.BLModule)
        };

        _pigeon = new PigeonIMU(DrivetrainConstants.pigeonId); // We need the talon; not anymore

        this._odometry = new SwerveDriveOdometry(DrivetrainConstants.kinematics, getRotation2d(),
                getSwerveModulePositions());

        m_logX = new DoubleLogEntry(log, "/drivetrain/position/real_x");
        m_logY = new DoubleLogEntry(log, "/drivetrain/position/real_y");
        m_logR = new DoubleLogEntry(log, "/drivetrain/position/real_rotation");
        m_logwV = new DoubleLogEntry(log, "/drivetrain/calibration/wanted_v");
        m_logV = new DoubleLogEntry(log, "/drivetrain/calibration/real_v");

        vision_xController = new PIDController(LimelightConstants.xGains.kP, LimelightConstants.xGains.kI,
                LimelightConstants.xGains.kD);
        vision_yController = new PIDController(LimelightConstants.yGains.kP, LimelightConstants.yGains.kI,
                LimelightConstants.yGains.kD);
        vision_thetaController = new PIDController(LimelightConstants.thetaGains.kP, LimelightConstants.thetaGains.kI,
                LimelightConstants.thetaGains.kD);

        restartControllers();

        initSDB();
    }

    private void initSDB() {
        SmartDashboard.putData("set angles", new InstantCommand(() -> setModulesAngle(0)));
        SmartDashboard.putNumber("drive kp", 0.0);
        SmartDashboard.putNumber("drive kd", 0.0);
        SmartDashboard.putNumber("steer kp", 0.0);
        SmartDashboard.putData("update pid", new InstantCommand(() -> updatePID()));
    }

    private void updatePID() {
        for (int i = 0; i < _modules.length; i++) {
            this._modules[i].setModulePID();
        }
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

        ChassisSpeeds speeds = DrivetrainConstants.kinematics.toChassisSpeeds(moduleStates);
        m_logwV.append(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        this._odometry.update(getRotation2d(), getSwerveModulePositions());

        updateSDB();
        updateTelemetry();
    }

    public void updateTelemetry() {
        Pose2d pose = _odometry.getPoseMeters();
        m_logX.append(pose.getX());
        m_logY.append(pose.getY());
        m_logR.append(pose.getRotation().getDegrees());
        m_logV.append(Math.abs(getDriveVelocity()));
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

        Pose2d pose = getPose();
        SmartDashboard.putNumber("xpos", pose.getX());
        SmartDashboard.putNumber("ypos", pose.getY());

        SmartDashboard.putNumber("rotation", getRotation2d().getRadians());
    }

    public Pose2d getPose() {
        // System.out.println("getting pose at:" + Timer.getFPGATimestamp());
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
        vision_thetaController.reset();

        vision_xController.setTolerance(LimelightConstants.xTol);
        vision_yController.setTolerance(LimelightConstants.yTol);
        vision_thetaController.setTolerance(LimelightConstants.thetaTol);

        vision_xController.setSetpoint(0);
        vision_yController.setSetpoint(0);
        vision_thetaController.setSetpoint(DrivetrainConstants.installAngle.getDegrees());
    }

    public void setVisionPIDsByInputs(double xp, double xi, double xd, double yp, double yi, double yd, double tp,
            double ti, double td) {
        vision_xController.setP(xp);
        vision_xController.setI(xi);
        vision_xController.setD(xd);

        vision_yController.setP(yp);
        vision_yController.setI(yi);
        vision_yController.setD(yd);

        vision_thetaController.setP(tp);
        vision_thetaController.setI(ti);
        vision_thetaController.setD(td);
    }

    public void driveByVisionControllers(double Xangle, double Yangle) {
        double x = 0;
        double y = 0;
        double t = vision_thetaController.calculate(this.getPose().getRotation().getDegrees());

        if (Math.abs(this.getPose().getRotation().getDegrees()
                - DrivetrainConstants.installAngle.getDegrees()) < LimelightConstants.spinToleranceDegrees) {
            // Don't move until we're somewhat aligned
            x = vision_xController.calculate(Xangle);
            y = vision_yController.calculate(Yangle);
        }

        this.drive(x, y, t, true);
    }

    public void setVisionPIDFromSDB() {
        this.setVisionPIDsByInputs(SmartDashboard.getNumber("xKP", 0), 0, 0, SmartDashboard.getNumber("yKP", 0), 0, 0,
                SmartDashboard.getNumber("tKP", 0), 0, 0);
    }

    public void visionInitSDB() {
        SmartDashboard.putNumber("xKP", 0);
        SmartDashboard.putNumber("yKP", 0);
        SmartDashboard.putNumber("tKP", 0);
        SmartDashboard.putData("update vision SDB", new InstantCommand(() -> this.setVisionPIDFromSDB()));
    }

    public double getDriveVelocity() {
        return _modules[0].getDriveVelocity();
    }

    public void setDrivePercent(double percent) {
        _modules[0].setDrivePercent(percent);
        _modules[1].setDrivePercent(percent);
        _modules[2].setDrivePercent(percent);
        _modules[3].setDrivePercent(percent);
    }

    public void logSpeedsError(ChassisSpeeds speeds) {
        ChassisSpeeds realSpeeds = DrivetrainConstants.kinematics.toChassisSpeeds(
                _modules[0].getState(), _modules[1].getState(), _modules[2].getState(), _modules[3].getState());

        System.out.printf("%f,%f,%f,%f,%f,%f\n",
                state.timeSeconds,
                state.velocityMetersPerSecond,
                Math.hypot(realSpeeds.vxMetersPerSecond,
                        realSpeeds.vyMetersPerSecond),
                // speeds.vxMetersPerSecond,
                // speeds.vyMetersPerSecond,
                // speeds.omegaRadiansPerSecond,
                // realSpeeds.vxMetersPerSecond,
                // realSpeeds.vyMetersPerSecond,
                // realSpeeds.omegaRadiansPerSecond,
                posErrorX,
                posErrorY,
                rotationError);
    }

    public void logPosError(Translation2d translation, Rotation2d rotation) {
        posErrorX = translation.getX();
        posErrorY = translation.getY();
        rotationError = rotation.getDegrees();
    }

    public void logState(PathPlannerState state) {
        this.state = state;
    }
}
