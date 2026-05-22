package frc.robot.mechanisms.Intake.IntakeRollers;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.Voltage;

import frc.robot.mechanisms.Intake.IntakeRollers.IntakeRollersIO.IntakeRollersIOInputs;

public class IntakeRollers extends Mechanism {

    private final IntakeRollersIO io;
    private IntakeRollersIOInputs inputs = new IntakeRollersIOInputs();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()));
    }

    public Command runIntake() {
        return runVoltageCommand(() -> IntakeRollersConstants.INTAKE_VOLTAGE)
                .named("IntakeRollers/Intake");
    }

    public Command runEject() {
        return runVoltageCommand(() -> IntakeRollersConstants.EJECT_VOLTAGE)
                .named("IntakeRollers/Eject");
    }

    public Command runIdle() {
        return runVoltageCommand(() -> IntakeRollersConstants.IDLE_VOLTAGE)
                .named("IntakeRollers/Idle");
    }

}
