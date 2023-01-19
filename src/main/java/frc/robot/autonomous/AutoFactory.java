package frc.robot.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * AutoFactory
 */
public class AutoFactory {

    private static SwerveAutoBuilder _autoBuilder;

    private static HashMap<String, Command> _eventMap = new HashMap<>();

    static {
        // add event markers here
        // for example:
        // _eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // _eventMap.put("intakeDown", new IntakeDown());
    }

    public static CommandBase createAuto(Drivetrain drivetrain, String pathName) {
        if (_autoBuilder == null) {
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
        }
        PathPlannerTrajectory pathGroup = PathPlanner.loadPath(pathName,
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared));
        return _autoBuilder.fullAuto(pathGroup);
    }
}