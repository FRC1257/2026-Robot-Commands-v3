package frc.robot.Mechanisms.Shooter.Flywheel;

import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import frc.robot.Mechanisms.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;

public class Flywheel extends Mechanism {

    private final FlywheelIO io;
    private FlywheelIOInputs inputs = new FlywheelIOInputs();

    private AngularVelocity goalVelocity = RadiansPerSecond.zero();

    public Flywheel(FlywheelIO io) {
        this.io = io;

        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    public Trigger atGoalVelocity() {
        return new Trigger(() -> inputs.leaderVelocityRadsPerSec.isNear(goalVelocity, FlywheelConstants.VELOCITY_TOLERANCE));
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()))
                .whenCanceled(io::stop);
    }

    private NeedsNameBuilderStage runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runRepeatedly(() -> {
            goalVelocity = velocity.get();
            io.setVelocity(velocity.get());
        }).whenCanceled(() -> {
            goalVelocity = RadiansPerSecond.zero();
            io.stop();
        });
    }

    public Command runClosePresetVelocity() {
        return runVelocityCommand(() -> FlywheelConstants.CLOSE_PRESET)
                .named("Flywheel/ClosePresetVelocity");
    }

    public Command runFarPresetVelocity() {
        return runVelocityCommand(() -> FlywheelConstants.FAR_PRESET)
                .named("Flywheel/FarPresetVelocity");
    }

    public Command runIdleVelocity() {
        return runVelocityCommand(() -> FlywheelConstants.IDLE_PRESET)
                .named("Flywheel/IdleVelocity");
    }
    
}
