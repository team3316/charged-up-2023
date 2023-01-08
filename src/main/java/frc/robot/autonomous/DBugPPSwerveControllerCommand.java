package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DBugPPSwerveControllerCommand extends CommandBase {
    private final boolean DEBUG = false;

    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final Supplier<Rotation2d> m_desiredRotation;

    public DBugPPSwerveControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Supplier<Rotation2d> desiredRotation,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(
                xController,
                yController,
                thetaController);

        m_outputModuleStates = outputModuleStates;

        m_desiredRotation = desiredRotation;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        if (DEBUG)
            System.out.printf("time,xError,xP,xF,yError,yP,yF,aError,aPF\n");
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        Pose2d currentPose = m_pose.get();

        var targetChassisSpeeds = m_controller.calculate(currentPose, desiredState, m_desiredRotation.get());

        if (DEBUG) {
            // Calculate back the constituate parts of the chassis speeds.
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds.vxMetersPerSecond,
                    targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond,
                    currentPose.getRotation().times(-1));
            Pose2d desiredPose = desiredState.poseMeters;
            double xFF = desiredState.velocityMetersPerSecond * desiredPose.getRotation().getCos();
            double yFF = desiredState.velocityMetersPerSecond * desiredPose.getRotation().getSin();
            System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                    curTime,
                    desiredPose.getX() - currentPose.getX(),
                    fieldSpeeds.vxMetersPerSecond - xFF,
                    xFF,
                    desiredPose.getY() - currentPose.getY(),
                    fieldSpeeds.vyMetersPerSecond - yFF,
                    yFF,
                    desiredState.holonomicRotation.getRadians() - currentPose.getRotation().getRadians(),
                    fieldSpeeds.omegaRadiansPerSecond);
        }

        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_outputModuleStates.accept(m_kinematics.toSwerveModuleStates(new ChassisSpeeds())); // stop drivetrain
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
