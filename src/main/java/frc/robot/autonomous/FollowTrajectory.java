// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectory {

    private final Drivetrain m_drivetrain;
    private PathPlannerTrajectory m_trajectory;
    private Supplier<Pose2d> _pose;
    private SwerveDriveKinematics _kinematics;
    private PIDController _xController;
    private PIDController _yController;
    private ProfiledPIDController _thetaController;
    private Consumer<SwerveModuleState[]> _outputModuleStates;

    public FollowTrajectory(Drivetrain drivetrain, String path) {
        this.m_drivetrain = drivetrain;
        this.m_trajectory = PathPlanner.loadPath(path,
                AutonomousConstants.kMaxSpeedMetersPerSecond,
                AutonomousConstants.kMaxAccelerationMetersPerSecondSquared);
        this._pose = m_drivetrain::getPose;
        this._kinematics = DrivetrainConstants.kinematics;
        this._xController = new PIDController(AutonomousConstants.kPXController, 0, 0);
        this._yController = new PIDController(AutonomousConstants.kPYController, 0, 0);
        this._thetaController = new ProfiledPIDController(AutonomousConstants.kPThetaController, 0, 0,
                AutonomousConstants.kThetaControllerConstraints);
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this._outputModuleStates = this.m_drivetrain::setDesiredStates;
    }

    public Command getFollowTrajectoryCommand() {

        return new DBugPPSwerveControllerCommand(m_trajectory,
                _pose, _kinematics,
                _xController, _yController,
                _thetaController,
                _outputModuleStates,
                () -> m_trajectory.getEndState().holonomicRotation,
                m_drivetrain);
    }

    public Command getResetOddometryCommand() {
        PathPlannerState initState = m_trajectory.getInitialState();

        return new InstantCommand(() -> {
            m_drivetrain.resetOdometry(
                    new Pose2d(initState.poseMeters.getTranslation(), initState.holonomicRotation));
        });
    }
}
