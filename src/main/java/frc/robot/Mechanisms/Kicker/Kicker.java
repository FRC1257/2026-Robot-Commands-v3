package frc.robot.mechanisms.Kicker;

import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import frc.robot.mechanisms.Kicker.KickerIO.KickerIOInputs;

public class Kicker extends Mechanism {
    
    private final KickerIO io;
    private KickerIOInputs inputs = new KickerIOInputs();

    private AngularVelocity goalVelocity = RadiansPerSecond.zero();

    public Kicker(KickerIO io) {
        this.io = io;
        
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    public Trigger atGoalVelocity() {
        return new Trigger(() -> inputs.leaderVelocityRadsPerSec.isNear(goalVelocity, KickerConstants.VELOCITY_TOLERANCE));
    }

    public Trigger isJammed() {
        return new Trigger(
            () -> inputs.leaderVelocityRadsPerSec.lte(KickerConstants.KICKER_JAMMED_VELOCITY) 
            && inputs.leaderCurrentAmps.gte(KickerConstants.KICKER_JAMMED_CURRENT)
        ).debounce(KickerConstants.KICKER_JAMMED_TIME, DebounceType.kRising);
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()));
    }

    private NeedsNameBuilderStage runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runRepeatedly(() -> {
            goalVelocity = velocity.get();
            io.setVelocity(velocity.get());
        });
    }

    public Command runEject() {
        return runVelocityCommand(() -> KickerConstants.EJECT_VELOCITY)
                .named("Kicker/Eject");
    }

    public Command runIntake() {
        return runVelocityCommand(() -> KickerConstants.INTAKE_VELOCITY)
                .named("Kicker/Intake");
    }

    public Command runIdle() {
        return runVelocityCommand(() -> KickerConstants.IDLE_VELOCITY)
                .named("Kicker/Idle");
    }

    public Command runUnjam() {
        return runVelocityCommand(() -> KickerConstants.UNJAM_VELOCITY)
                .until(isJammed().negate())
                .withPriority(Command.HIGHEST_PRIORITY)
                .named("Kicker/Unjam");
    }
        
}
