package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.motors.PIDFGains;

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

    private SimpleMotorFeedforward goToDirectionFeedForward;

    private ProfiledPIDController goToDirectionController;
    private boolean goingToDirection = false;

    public static enum DIRECTIONS {
        FORWARD(90),
        BACKWARD(-90),
        RIGHT(0),
        LEFT(180);

        public final double angleDeg;

        private DIRECTIONS(double angleDeg) {
            this.angleDeg = angleDeg;
        }
    }

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

        vision_xController = new PIDController(LimelightConstants.xKp, 0, 0);
        vision_yController = new PIDController(LimelightConstants.yKp, 0, 0);
        thetaController = new PIDController(LimelightConstants.thetaKp, 0, 0);
        vision_xController.setTolerance(LimelightConstants.xTol);
        vision_yController.setTolerance(LimelightConstants.yTol);
        thetaController.setTolerance(LimelightConstants.thetaTol);
        vision_xController.setSetpoint(0); // arbitrary. Overriedden when setting internal state.
        vision_yController.setSetpoint(0);
        thetaController.setSetpoint(DrivetrainConstants.installAngle.getDegrees());

        goToDirectionController = new ProfiledPIDController(DrivetrainConstants.goToDirectionKp, 0, 0,
                DrivetrainConstants.goToDirectionConstrains);
        goToDirectionFeedForward = new SimpleMotorFeedforward(DrivetrainConstants.goToDirectionKs, 
        DrivetrainConstants.goToDirectionKv, 
        DrivetrainConstants.goToDirectionKa);

        resetControllers();
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

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d rightJoystick) {
        DIRECTIONS targetDirection = null;
        double rightJoystickMag = rightJoystick.getNorm();
        double rightJoystickAngleDeg = rightJoystick.getAngle().getDegrees();

        if (rightJoystickMag >= DrivetrainConstants.rightJoystickDeadband) {
            for (DIRECTIONS d : DIRECTIONS.values()) {
                if (d == DIRECTIONS.LEFT) {
                    rightJoystickAngleDeg = Math.abs(rightJoystickAngleDeg);
                }
                if (Math.abs(d.angleDeg - rightJoystickAngleDeg) <= DrivetrainConstants.rightJoystickAngleErrorDeg) {
                    targetDirection = d;
                    break;
                }
            }
        }

        SmartDashboard.putNumber("Joystick Mag", rightJoystickMag);
        SmartDashboard.putNumber("Joystick Angle", rightJoystickAngleDeg);

        if (targetDirection != null) {
            SmartDashboard.putString("DIRECTION", targetDirection.toString());
            if (goToDirectionController.getGoal().position != targetDirection.angleDeg) {
                goToDirectionController.setGoal(targetDirection.angleDeg);
            }

            rot = goToDirectionFeedForward.calculate(goToDirectionController.getSetpoint().position, 
            goToDirectionController.getSetpoint().velocity) + goToDirectionController.calculate(getHeading());
            if (goToDirectionController.atGoal()) {
                targetDirection = null;
            }
        } else {
            SmartDashboard.putString("DIRECTION", "none");
        }
        drive(xSpeed, ySpeed, rot, fieldRelative);
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

        updateSDB();
        SmartDashboard.putNumber("pitch", this.getPitch());
        // printEverything();
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

    @SuppressWarnings({ "unused" })
    private void printEverything() {
        String printString = new String();
        printString += Timer.getFPGATimestamp() + ",";
        ChassisSpeeds speeds = DrivetrainConstants.kinematics.toChassisSpeeds(_modules[0].getState(),
                _modules[1].getState(),
                _modules[2].getState(), _modules[3].getState());
        printString += speeds.vxMetersPerSecond + "," + speeds.vyMetersPerSecond + ",";
        for (int i = 0; i < this._modules.length; i++) {
            printString += ",speed," + _modules[i].getState().speedMetersPerSecond;
            printString += ",angle," + _modules[i].getState().angle.getDegrees();
            printString += ",desSpeed," + _modules[i].getTargetState().speedMetersPerSecond;
            printString += ",desAngle," + _modules[i].getTargetState().angle.getDegrees();
        }

        System.out.println(printString);

    }

    public void resetControllers() {
        vision_xController.reset();
        vision_yController.reset();
        thetaController.reset();
    }

    public void setXSetpoint(double xGoal) {
        vision_xController.setSetpoint(xGoal);
    }

    public void driveByVisionControllers(double xDistance, double yDistance, boolean hasTarget) {
        double x = 0;
        double y = 0;
        double t = thetaController.calculate(this.getPose().getRotation().getDegrees());

        if (Math.abs(this.getPose().getRotation().getDegrees()
                - DrivetrainConstants.installAngle.getDegrees()) < LimelightConstants.spinToleranceDegrees
                && hasTarget) {
            x = vision_xController.calculate(xDistance);
            y = vision_yController.calculate(yDistance);
        }

        this.drive(vision_xController.atSetpoint() ? 0 : x, vision_yController.atSetpoint() ? 0 : y, t, true);
    }

    public void setKeepHeading(Rotation2d rotation) {
        thetaController.reset();
        thetaController.setTolerance(LimelightConstants.thetaTol);
        thetaController.setSetpoint(rotation.getRadians());
    }

    public void driveAndKeepHeading(double xSpeed, double ySpeed) {
        // the setpoint is set with `this::setKeepHeading`
        this.drive(xSpeed, ySpeed, thetaController.calculate(this.getPose().getRotation().getRadians()), true);
    }

    public double getPitch() {
        return _pigeon.getPitch();
    }

    public CommandBase getRotateModulesCommand() {
        return new RunCommand(() -> drive(0, -0.1, 0, false)).withTimeout(0.2)
                .finallyDo((interrupted) -> drive(0, 0, 0, false));
    }
}
