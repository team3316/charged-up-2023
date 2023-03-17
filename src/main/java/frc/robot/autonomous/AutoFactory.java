package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * AutoFactory
 */
public class AutoFactory {

    private SwerveAutoBuilder _autoBuilder;

    private HashMap<String, Command> _eventMap = new HashMap<>();

    public AutoFactory(Drivetrain drivetrain, AutoRollerGripper rollerGripper) {
        _autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::resetPose,
                DrivetrainConstants.kinematics,
                new PIDConstants(AutonomousConstants.kPTranslationController, 0.0, 0.0),
                new PIDConstants(AutonomousConstants.kPThetaController, 0.0, 0.0),
                drivetrain::setDesiredStates,
                _eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                drivetrain);

        // add event markers here (and add the subsystem to the constructor)
        _eventMap.put("engage_stop", new InstantCommand(() -> drivetrain.setModulesAngle(90)));
        _eventMap.put("eject", rollerGripper.getEjectCommand());
        _eventMap.put("intake", rollerGripper.getIntakeCommand());

    }

    public CommandBase createAuto(String pathName, PathConstraints... extraConstraints) {
        List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared),
                extraConstraints);
        return _autoBuilder.fullAuto(paths);
    }

    public CommandBase createfollow(PathPlannerTrajectory path) {
        return _autoBuilder.followPath(path);
    }

}