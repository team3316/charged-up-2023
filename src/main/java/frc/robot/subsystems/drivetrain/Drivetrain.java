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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
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

    public void driveAndSpinByState(TrapezoidProfile.State spinState, double kP) {
        double PIDOutput = kP * (spinState.position - getRotation2d().getRadians());
        drive(0, 0, spinState.velocity + PIDOutput, true);

        // SmartDashboard.putNumber("profile position", spinState.position);
        // SmartDashboard.putNumber("profile velocity", spinState.velocity);
        // SmartDashboard.putNumber("pid output", PIDOutput);

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

    public Command getSpinToAngleCommand(Rotation2d goal) {
        return new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new Constraints(DrivetrainConstants.maxAngularVelocityRadsPerSec,
                                DrivetrainConstants.maxAngularaccRadsPerSecSquared),
                        new State(goal.getRadians(), 0),
                        new State(getRotation2d().getRadians(), 0)),
                (State state) -> this.driveAndSpinByState(state, SmartDashboard.getNumber("kP", 0)),
                this);
    }

    public Command getSpinByInputCommand(DoubleSupplier input, DoubleSupplier goal) {
        return new PIDCommand(new PIDController(0.1, 0, 0), input, goal,
                (double output) -> drive(0, output, 0, false));
    }
}