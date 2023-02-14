package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * AutoFactory
 */
public class AutoFactory {

    private SwerveAutoBuilder _autoBuilder;

    private HashMap<String, Command> _eventMap = new HashMap<>();

    public AutoFactory(Drivetrain drivetrain, DataLog log) {
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

    }

    public CommandBase createAuto(String pathName) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared));
        return _autoBuilder.fullAuto(path);
    }

    public CommandBase createfollow(PathPlannerTrajectory path) {
        return _autoBuilder.followPath(path);
    }

}