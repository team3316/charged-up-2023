package frc.robot.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * AutoFactory
 */
public class AutoFactory {

    private SwerveAutoBuilder _autoBuilder;

    private HashMap<String, Command> _eventMap = new HashMap<>();

    private DoubleLogEntry _logX, _logY, _logR;

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

        this._logX = new DoubleLogEntry(log, "/drivetrain/position/desired_x");
        this._logY = new DoubleLogEntry(log, "/drivetrain/position/desired_y");
        this._logR = new DoubleLogEntry(log, "/drivetrain/position/desired_rotation");

        // add event markers here (and add the subsystem to the constructor)
        // for example:
        // _eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // _eventMap.put("intakeDown", new IntakeDown());
    }

    public CommandBase createAuto(Drivetrain drivetrain, String pathName) {
        Timer time = new Timer();
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName,
                new PathConstraints(AutonomousConstants.kMaxSpeedMetersPerSecond,
                        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared));
        return _autoBuilder.fullAuto(path).alongWith(new FunctionalCommand(
                () -> time.start(),
                () -> {
                    this._logX.append(path.sample(time.get()).poseMeters.getX());
                    this._logY.append(path.sample(time.get()).poseMeters.getY());
                    this._logR.append(path.sample(time.get()).poseMeters.getRotation().getDegrees());
                },
                (interrupted) -> {
                    time.stop();
                    time.reset();
                },
                () -> false));
    }

    public CommandBase createfollow(PathPlannerTrajectory path) {
        return _autoBuilder.followPath(path);
    }
}