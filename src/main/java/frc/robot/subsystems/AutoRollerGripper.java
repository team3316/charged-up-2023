// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoRollerGripper extends SubsystemBase {
    DigitalInput isEquipped;
    TalonFX leaderT, followerT;
    CANSparkMax folderSM;

    double _speed, _ms;

    public AutoRollerGripper(double voltage, double ms, int talonPort) {
        leaderT = new TalonFX(talonPort);
        followerT.setInverted(true);

    }

    public CommandBase getCollectCommand() {
        return new CommandBase() {

        };
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
