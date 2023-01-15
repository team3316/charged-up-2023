package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoRollerGripper;
import frc.robot.subsystems.AutoRollerGripper.RollersState;;


public class AutoRollerGripperCommand extends CommandBase{
    private final AutoRollerGripper m_rollergripper;
    
    public AutoRollerGripperCommand(AutoRollerGripper subsystem) {
        this.m_rollergripper = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
