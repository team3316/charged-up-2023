package frc.robot.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.AutoRollerGripper.FolderState;
import frc.robot.subsystems.AutoRollerGripper.RollersState;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * AutoFactory
 */
public class AutoFactory {

    private SwerveAutoBuilder _autoBuilder;

    private HashMap<String, Command> _eventMap = new HashMap<>();

    public AutoFactory(Drivetrain drivetrain, AutoRollerGripper autoRollerGripper) {
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
        // for example:
        // _eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // _eventMap.put("intakeDown", new IntakeDown());
        _eventMap.put("intake",
                autoRollerGripper.getFoldCommand(FolderState.OUT) // nothing will happen if it's in the correct state
                        .andThen(autoRollerGripper.getIntakeCommand()));

        _eventMap.put("eject",
                autoRollerGripper.getFoldCommand(FolderState.OUT) // for first eject
                        .andThen(autoRollerGripper.getEjectCommand()));

    }

    public CommandBase createAuto(String pathName) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName,
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared));
        return _autoBuilder.fullAuto(path);
    }
}