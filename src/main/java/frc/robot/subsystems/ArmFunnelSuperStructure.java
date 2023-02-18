package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Funnel.FunnelPosition;

public class ArmFunnelSuperStructure {
    private final Arm m_arm;
    private final Funnel m_funnel;

    public ArmFunnelSuperStructure(Arm arm, Funnel funnel) {
        m_arm = arm;
        m_funnel = funnel;
    }

    public CommandBase getSetStateCommand(ArmState arm, FunnelPosition funnel) {
        return new InstantCommand();
    }
}
