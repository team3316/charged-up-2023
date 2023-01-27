package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private DoubleLogEntry m_logX, m_logY, m_logR, m_logLLx, m_logLLy, m_logLLrot;

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

    public Command getLimeLightAllignCommand(DoubleSupplier inputX, DoubleSupplier inputY) {
        PIDController xControl = new PIDController(SmartDashboard.getNumber("xKP", 0), LimelightConstants.xKI,
                LimelightConstants.xKD);
        PIDController yControl = new PIDController(SmartDashboard.getNumber("yKP", 0), LimelightConstants.yKI,
                LimelightConstants.yKD);
        PIDController thetaControl = new PIDController(SmartDashboard.getNumber("tKP", 0), LimelightConstants.thetaKI,
                LimelightConstants.thetaKD);

        xControl.reset();
        yControl.reset();
        thetaControl.reset();

        xControl.setTolerance(LimelightConstants.xTol);
        yControl.setTolerance(LimelightConstants.yTol);
        thetaControl.setSetpoint(LimelightConstants.thetaTol);

        xControl.setSetpoint(0);
        yControl.setSetpoint(0);
        thetaControl.setSetpoint(LimelightConstants.installAngle.getDegrees());

        return new RunCommand(
                () -> drive(-yControl.calculate(inputY.getAsDouble()),
                        xControl.calculate(inputX.getAsDouble() + this.getPose().getRotation().getDegrees()
                                + LimelightConstants.limelightRotations.getDegrees()),
                        thetaControl.calculate(this.getPose().getRotation().getDegrees()), true),
                this)
                .until(() -> (xControl.atSetpoint() && yControl.atSetpoint() && thetaControl.atSetpoint()))
                .andThen(() -> {
                    xControl.close();
                    yControl.close();
                    thetaControl.close();
                });
    }
}
