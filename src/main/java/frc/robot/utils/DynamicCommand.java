package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DynamicCommand extends CommandBase {
    Supplier<CommandBase> _commandConstructor;
    CommandBase _command;

    public DynamicCommand(Supplier<CommandBase> commandConstructor, SubsystemBase... requirements) {
        _commandConstructor = commandConstructor;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        _command = _commandConstructor.get();
        _command.initialize();
    }

    @Override
    public void execute() {
        _command.execute();
    }

    @Override
    public boolean isFinished() {
        return _command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _command.end(interrupted);
    }

}
