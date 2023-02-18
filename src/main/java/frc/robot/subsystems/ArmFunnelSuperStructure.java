package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
        CommandBase armMovementSequence = new InstantCommand();
        if (arm == m_arm.getTargetState()) {
            switch (arm) {
                case COLLECT: // moving to COLLECT from not COLLECT
                    armMovementSequence = Commands.sequence(
                            m_arm.getSetStateCommand(ArmState.DRIVE),
                            m_funnel.setFunnelPositionCommand(FunnelPosition.OPEN),
                            m_arm.getSetStateCommand(ArmState.COLLECT));
                    break;

                case DRIVE:
                    if (m_arm.getTargetState() == ArmState.COLLECT) // moving to DRIVE to COLLECT
                    {
                        armMovementSequence = Commands.sequence(
                                m_funnel.setFunnelPositionCommand(FunnelPosition.OPEN),
                                m_arm.getSetStateCommand(ArmState.DRIVE));
                    } else
                        armMovementSequence = Commands.sequence(m_arm.getSetStateCommand(ArmState.DRIVE)); // moving
                                                                                                           // from SCORE
                                                                                                           // to DRIVE

                    break;

                default:
                    if (m_arm.getTargetState() == ArmState.COLLECT) { // moving from COLLECT to SCORE
                        armMovementSequence = Commands.sequence(
                                m_funnel.setFunnelPositionCommand(FunnelPosition.OPEN),
                                m_arm.getSetStateCommand(ArmState.DRIVE),
                                m_funnel.setFunnelPositionCommand(FunnelPosition.CLOSED),
                                m_arm.getSetStateCommand(arm));
                    } else
                        armMovementSequence = Commands.sequence( // moving from DRIVE/SCORE to different SCORE
                                m_funnel.setFunnelPositionCommand(FunnelPosition.CLOSED), // safety
                                m_arm.getSetStateCommand(arm));

            }
        }

        if (funnel != m_funnel.getFunnelPosition())
            return Commands.sequence(armMovementSequence, m_funnel.setFunnelPositionCommand(funnel));
        return armMovementSequence;

    }

    public void stop() {
        m_arm.stop();
        m_funnel.stop();
    }
}
