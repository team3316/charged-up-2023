// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FunnelConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Funnel extends SubsystemBase {

  private CANSparkMax _rollers;
  private FunnelState _currentState;
  private DoubleSolenoid _funnelSolenoid;

  public enum FunnelState {
    COLLECT(FunnelConstants.openState, FunnelConstants.collectPercent),
    INSTALL(FunnelConstants.openState, FunnelConstants.installPercent),
    CLOSED(FunnelConstants.closedState, FunnelConstants.closedPercent);

    public final DoubleSolenoid.Value solenoidState;
    public final double rollerPercent;

    private FunnelState(Value solenoidState, double rollerPercent) {
      this.solenoidState = solenoidState;
      this.rollerPercent = rollerPercent;
    }
  };

  public Funnel() {
    this._funnelSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, FunnelConstants.solenoidForwardPort,
        FunnelConstants.solenoidReversePort);

    // We only use this SparkMax with Percent Output. So no need for all params
    this._rollers = DBugSparkMax.create(FunnelConstants.sparkMaxPort, new PIDFGains(0), 1, 1, 0);
  }

  public FunnelState getFunnelState() {
    return _currentState;
  }

  public void setFunnelState(FunnelState state) {
    if (state == getFunnelState()) {
      return;
    }

    _funnelSolenoid.set(state.solenoidState);
    _rollers.set(state.rollerPercent);

    _currentState = state;

    SmartDashboard.putString("Funnel State", this._currentState.toString());
  }

  public CommandBase setFunnelStateCommand(FunnelState state) {
    return new InstantCommand(() -> setFunnelState(state), this);
  }
}