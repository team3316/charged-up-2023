package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Funnel.FunnelPosition;
import frc.robot.subsystems.Funnel.FunnelRollersState;

public class ArmFunnelSuperStructure {
    private final Arm m_arm;
    private final Funnel m_funnel;

    public ArmFunnelSuperStructure(Arm arm, Funnel funnel) {
        m_arm = arm;
        m_funnel = funnel;
    }

    public CommandBase generateSetStateCommand(ArmState wantedArmState, FunnelPosition wantedFunnelState) {
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
            wantedFunnelState = FunnelPosition.CLOSED;

        if (wantedArmState == m_arm.getTargetState())
            return m_funnel.setFunnelPositionCommand(wantedFunnelState);
        if (m_arm.getTargetState() == ArmState.COLLECT)
            return Commands.sequence(
                    m_funnel.setFunnelPositionCommand(FunnelPosition.OPEN),
                    m_arm.getSetStateCommand(ArmState.DRIVE),
                    m_funnel.setFunnelPositionCommand(FunnelPosition.CLOSED),
                    m_arm.getSetStateCommand(wantedArmState));

        if (wantedArmState == ArmState.COLLECT)
            return Commands.sequence(
                    m_arm.getSetStateCommand(ArmState.DRIVE),
                    m_funnel.setFunnelPositionCommand(FunnelPosition.OPEN), m_arm.getSetStateCommand(ArmState.COLLECT),
                    m_funnel.setFunnelPositionCommand(wantedFunnelState));

        return m_funnel.setFunnelPositionCommand(wantedFunnelState).andThen(m_arm.getSetStateCommand(wantedArmState));
    }

    public CommandBase setFunnelRollersStateCommand(FunnelRollersState state) {
        return m_funnel.setFunnelRollersStateCommand(state);
    }

    public CommandBase getSetStateCommand(ArmState wantedArmState, FunnelPosition wantedFunnelState) {
        // return new DynamicCommand(() -> generateSetStateCommand(wantedArmState,
        // wantedFunnelState));
        return new InstantCommand(() -> generateSetStateCommand(wantedArmState, wantedFunnelState).schedule())
                .until(() -> (Math.abs(m_arm.getAngle() - wantedArmState.stateAngle) <= 0.1));
    }

    public void stop() {
        m_arm.stop();
        m_funnel.stop();
    }
}
