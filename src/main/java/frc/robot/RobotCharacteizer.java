package frc.robot;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotCharacteizer {
    private final DoubleConsumer setPercent;
    private final DoubleSupplier getVelociy;
    private final DoubleArrayLogEntry mLog;
    private final SubsystemBase drivetrain;

    private static final double LOW_ACCELERATION = 0.1; // % per second. 0.1 will take 10 seconds to reach 100% (1.0)
    private static final double HIGH_ACCELERATION = 1.0;

    private double startTime;

    RobotCharacteizer(DoubleConsumer setPercent, DoubleSupplier getVelociy, SubsystemBase drivetrain) {
        this.setPercent = setPercent;
        this.getVelociy = getVelociy;
        this.mLog = new DoubleArrayLogEntry(DataLogManager.getLog(), "/drivetrain/calibrate");
        this.drivetrain = drivetrain;
    }

    public void log(double time, double percent) {
        double[] record = { time, percent, this.getVelociy.getAsDouble() };
        mLog.append(record);
    }

    private void stop() {
        this.setPercent.accept(0.0);
    }

    public CommandBase getLowAccelerationCommand(boolean forward) {
        return new FunctionalCommand(
                () -> {
                    stop();
                    startTime = Timer.getFPGATimestamp();
                },
                () -> {
                    double time = Timer.getFPGATimestamp() - this.startTime;
                    double percent = time * RobotCharacteizer.LOW_ACCELERATION * (forward ? 1 : -1);
                    setPercent.accept(percent);
                    log(time, percent);
                },
                interrupted -> stop(),
                () -> false, drivetrain);
    }

    public CommandBase getHighAccelerationCommand(boolean forward) {
        return new FunctionalCommand(
                () -> {
                    setPercent.accept(RobotCharacteizer.HIGH_ACCELERATION * (forward ? 1 : -1));
                    startTime = Timer.getFPGATimestamp();
                },
                () -> {
                    double time = Timer.getFPGATimestamp() - this.startTime;
                    double percent = RobotCharacteizer.HIGH_ACCELERATION * (forward ? 1 : -1);
                    log(time, percent);
                },
                interrupted -> stop(),
                () -> false, drivetrain);
    }
}
