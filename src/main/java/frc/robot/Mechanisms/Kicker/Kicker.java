package frc.robot.Mechanisms.Kicker;

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

import frc.robot.Mechanisms.Kicker.KickerIO.KickerIOInputs;

public class Kicker extends Mechanism {
    
    private final KickerIO io;
    private KickerIOInputs inputs = new KickerIOInputs();

    private AngularVelocity goalVelocity = RadiansPerSecond.zero();

    private final Debouncer unjamDebouncer = new Debouncer(0.5, DebounceType.kRising);

    public Kicker(KickerIO io) {
        this.io = io;
        
        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    public Trigger atGoalVelocity() {
        return new Trigger(() -> inputs.leaderVelocityRadsPerSec.isNear(goalVelocity, KickerConstants.VELOCITY_TOLERANCE));
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

    public Command runEjectPreset() {
        return runVelocityCommand(() -> KickerConstants.EJECT_PRESET)
                .named("Kicker/EjectPreset");
    }

    public Command runIntakePreset() {
        return runVelocityCommand(() -> KickerConstants.INTAKE_PRESET)
                .named("Kicker/IntakePreset");
    }

    public Command runUnjamPreset() {
        return runVelocityCommand(() -> KickerConstants.UNJAM_PRESET)
                .until(() -> unjamDebouncer.calculate(inputs.leaderCurrentAmps.lt(KickerConstants.UNJAM_CURRENT_THRESHOLD)))
                .withPriority(Command.HIGHEST_PRIORITY)
                .named("Kicker/UnjamPreset");
    }
        
}
