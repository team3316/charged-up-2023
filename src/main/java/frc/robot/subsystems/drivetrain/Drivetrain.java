package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autonomous.AutoFactory;
import frc.robot.constants.AutonomousConstants;
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
    private static PIDController vision_thetaController;
    private static PIDController spin_Controller;

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

        vision_xController = new PIDController(LimelightConstants.xGains.kP, LimelightConstants.xGains.kI,
                LimelightConstants.xGains.kD);
        vision_yController = new PIDController(LimelightConstants.yGains.kP, LimelightConstants.yGains.kI,
                LimelightConstants.yGains.kD);
        vision_thetaController = new PIDController(LimelightConstants.thetaGains.kP, LimelightConstants.thetaGains.kI,
                LimelightConstants.thetaGains.kD);

        spin_Controller = new PIDController(DrivetrainConstants.spinGains.kP, DrivetrainConstants.spinGains.kI,
                DrivetrainConstants.spinGains.kD);

        restartControllers();
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

    public Command getMoveByTranslation2dCommand(Translation2d GoalOffsetTrans, AutoFactory factory) {
        Translation2d currentTrans = new Translation2d(this.getPose().getX(), this.getPose().getY());
        PathPlannerTrajectory transTrajectory = PathPlanner.generatePath(
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathPoint(currentTrans,
                        LimelightConstants.installAngle, this.getPose().getRotation()), // if not calibrated, will be
                                                                                        // jittery (as in to dierction
                                                                                        // of
                // travel)
                new PathPoint(currentTrans.plus(GoalOffsetTrans),
                        LimelightConstants.installAngle, this.getPose().getRotation()));
        System.out.println("currentTrans " + currentTrans.getX() + ", " + currentTrans.getY());
        System.out.println("currentTrans " + currentTrans.plus(GoalOffsetTrans).getX() + ", "
                + currentTrans.plus(GoalOffsetTrans).getY());
        return factory.createfollow(transTrajectory);
    }

    public void restartControllers() {
        vision_xController.reset();
        vision_yController.reset();
        vision_thetaController.reset();
        spin_Controller.reset();

        vision_xController.setTolerance(LimelightConstants.xTol);
        vision_yController.setTolerance(LimelightConstants.yTol);
        vision_thetaController.setTolerance(LimelightConstants.thetaTol);
        spin_Controller.setTolerance(DrivetrainConstants.spinTol);

        vision_xController.setSetpoint(0);
        vision_yController.setSetpoint(0);
        vision_thetaController.setSetpoint(LimelightConstants.installAngle.getDegrees());
        spin_Controller.setSetpoint(LimelightConstants.installAngle.getDegrees());
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
        if (!this.atInstallAngle()) {
            this.spinToInstallAngle();
            return;
        }
        this.drive(vision_xController.calculate(Xangle),
                vision_yController.calculate(Yangle),
                vision_thetaController.calculate(this.getPose().getRotation().getDegrees()),
                true);
    }

    public boolean controllersAtSetpoint() {
        return vision_xController.atSetpoint() && vision_yController.atSetpoint()
                && vision_thetaController.atSetpoint();
    }

    public boolean atInstallAngle() {
        return spin_Controller.atSetpoint();
    }

    public void spinToInstallAngle() {
        this.drive(0, 0, spin_Controller.calculate(getPose().getRotation().getDegrees()), false);
    }

    public void visionPIDBySDB() {
        this.setVisionPIDsByInputs(SmartDashboard.getNumber("xKP", 0), 0, 0, SmartDashboard.getNumber("yKP", 0), 0, 0,
                SmartDashboard.getNumber("tKP", 0), 0, 0);
    }
}
