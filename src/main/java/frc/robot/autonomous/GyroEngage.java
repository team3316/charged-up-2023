package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.AutonomousConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Within;

public class GyroEngage extends CommandBase {

    private Drivetrain _drivetrain;
    private double _driveSpeed;

    public GyroEngage(Drivetrain drivetrain, double driveSpeed) {
        _drivetrain = drivetrain;
        addRequirements(_drivetrain);
        _driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {
        _drivetrain.drive(_driveSpeed, 0, 0, false);
    }

    @Override
    public void execute() {
        System.out.printf("%f,%f\n", Timer.getFPGATimestamp(), _drivetrain.getPitch());
    }

    @Override
    public boolean isFinished() {
        return Within.range(_drivetrain.getPitch(), 0, 10);
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.drive(0, 0, 0, false);
        _drivetrain.setModulesAngle(90);
    }
}
