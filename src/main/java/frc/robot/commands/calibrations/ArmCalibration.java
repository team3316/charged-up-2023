// package frc.robot.commands.calibrations;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.humanIO.PS5Controller;
// import frc.robot.subsystems.Arm;

// public class ArmCalibration extends CommandBase {

//     PS5Controller _controller;

//     ArmFeedforward _feedforward;

//     Subsystem _subsystem;
//     TalonFX _leader;
//     TalonFXConfiguration _leaderConfig;

//     public static boolean armClibrationSequenceEnded = false;

//     // ff gains
//     double ks = 0;
//     double kv = 0;
//     double kg = 0;
//     double ka = 0;

//     // pid gains
//     double kp = 0;
//     double ki = 0;
//     double kd = 0;

//     boolean isEnded = false;

//     double _currentValue;

//     public ArmCalibration(Arm subsystem) {
//         this._subsystem = subsystem;
//         addRequirements(this._subsystem);
//     }

    

    

//     public void init() {
        
//     }

//     public void execute() {
//     }

//     public CommandBase endSequence() {
//         return new InstantCommand(() -> isFinished(true));
//     }

//     public boolean isFinished(boolean end) {
//         return end;
//     }

//     public void end() {
//         _leader.set(TalonFXControlMode.PercentOutput, 0);
//     }
// }
