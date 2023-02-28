package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class GyroEngage extends CommandBase {

    private Drivetrain _drivetrain;
    private double _driveSpeed;
    private double _driveSetPoint;
    private boolean _isSetPointOver;

    public GyroEngage(Drivetrain drivetrain, double driveSpeed, double driveSetPoint, boolean isSetPointOver) {
        _drivetrain = drivetrain;
        addRequirements(_drivetrain);
        _driveSpeed = driveSpeed;
        _driveSetPoint = driveSetPoint;
        _isSetPointOver = isSetPointOver;
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
        if (_isSetPointOver)
            return _drivetrain.getPitch() > _driveSetPoint;
        return _drivetrain.getPitch() < _driveSetPoint;
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.drive(0, 0, 0, false);
    }
}
