package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Funnel.FunnelState;
import frc.robot.utils.DynamicCommand;

public class ArmFunnelSuperStructure {
    private final Arm m_arm;
    private final Funnel m_funnel;

    public ArmFunnelSuperStructure(Arm arm, Funnel funnel) {
        m_arm = arm;
        m_funnel = funnel;
    }

    public CommandBase generateSetStateCommand(ArmState wantedArmState, FunnelState wantedFunnelState) {
        /**
         * If arm in collect and moves out or arm is out and move to collect:
         * open -> drive -> closed -> continue
         * open -> collect -> final state -> continue
         * 
         * 1 collect-to-drive = sequence(open, drive, close)
         * 2 drive-to-collect = sequence(open, collect, final_state)
         * 
         * If arm is > drive
         * close -> drive -> continue
         * 
         * 3 closed-to = sequence(close, target)
         * 
         * collect drive mid cone mid cube low
         * collect x (1+3) (1+3) (1+3) (1+3)
         * drive (3+2) (3) (3) (3) (3)
         * mid cone (3+2) (3) (3) (3) (3)
         * mid cube (3+2) (3) (3) (3) (3)
         * low (3+2) (3) (3) (3) (3)
         * 
         * if (pos == target) return ();
         * else if (pos == collect) return (1+3);
         * else if (tar == collect) return (3+2);
         * else return 3
         * 
         */

        if (wantedArmState != ArmState.COLLECT)
            wantedFunnelState = FunnelState.CLOSED;

        if (wantedArmState == m_arm.getTargetState()) {
            // TODO: fix this case
            // if we press "stow" and while the arm is moving press "collect"
            // we want the arm to keep moving
            if (wantedArmState == ArmState.COLLECT && m_arm.getInitialState() != ArmState.COLLECT) {
                return Commands.sequence(
                        m_funnel.setFunnelStateCommand(FunnelState.OPEN),
                        m_arm.getSetStateCommand(ArmState.COLLECT),
                        m_funnel.setFunnelStateCommand(wantedFunnelState));
            }

            return m_funnel.setFunnelStateCommand(wantedFunnelState);
        }
        if (m_arm.getTargetState() == ArmState.COLLECT)
            return Commands.sequence(
                    m_funnel.setFunnelStateCommand(FunnelState.OPEN),
                    m_arm.getSetStateCommand(ArmState.DRIVE),
                    m_funnel.setFunnelStateCommand(FunnelState.CLOSED),
                    m_arm.getSetStateCommand(wantedArmState));

        if (wantedArmState == ArmState.COLLECT)
            return Commands.sequence(
                    m_arm.getSetStateCommand(ArmState.DRIVE),
                    m_funnel.setFunnelStateCommand(FunnelState.OPEN),
                    m_arm.getSetStateCommand(ArmState.COLLECT),
                    m_funnel.setFunnelStateCommand(wantedFunnelState));

        return m_funnel.setFunnelStateCommand(wantedFunnelState).andThen(m_arm.getSetStateCommand(wantedArmState));
    }

    public CommandBase getSetStateCommand(ArmState wantedArmState, FunnelState wantedFunnelState) {
        return new DynamicCommand(() -> generateSetStateCommand(wantedArmState, wantedFunnelState), m_arm, m_funnel);
    }

    public void stop() {
        m_arm.stop();
        m_funnel.stop();
    }

    public void init() {
        m_arm.changeTargetState(m_arm.getInitialState());
    }

    public CommandBase overrideCommand() {
        return Commands.sequence(m_funnel.setFunnelStateCommand(FunnelState.OPEN),
                m_arm.getSetStateCommand(ArmState.MID_CONE), m_funnel.setFunnelStateCommand(FunnelState.CLOSED));
    }
}
