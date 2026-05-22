package frc.robot.mechanisms.Indexer;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.Voltage;

import frc.robot.mechanisms.Indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends Mechanism {

    private final IndexerIO io;
    private IndexerIOInputs inputs = new IndexerIOInputs(); 

    public Indexer(IndexerIO io) {
        this.io = io;

        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()));
    }

    public Command runIntake() {
        return runVoltageCommand(() -> IndexerConstants.INTAKE_VOLTAGE)
                .named("Indexer/Intake");
    }

    public Command runEject() {
        return runVoltageCommand(() -> IndexerConstants.EJECT_VOLTAGE)
                .named("Indexer/Eject");
    }

    public Command runIdle() {
        return runVoltageCommand(() -> IndexerConstants.IDLE_VOLTAGE)
                .named("Indexer/Idle");
    }
    
}
