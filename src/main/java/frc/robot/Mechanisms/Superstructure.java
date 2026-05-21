package frc.robot.Mechanisms;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.units.measure.Distance;

import frc.robot.Mechanisms.Indexer.Indexer;
import frc.robot.Mechanisms.Kicker.Kicker;
import frc.robot.Mechanisms.Shooter.Flywheel.Flywheel;
import frc.robot.Mechanisms.Shooter.Hood.Hood;

public class Superstructure extends Mechanism {
    
    private final Indexer indexer;
    private final Kicker kicker;
    private final Flywheel flywheel;
    private final Hood hood;

    public Superstructure(Indexer indexer, Kicker kicker, Flywheel flywheel, Hood hood) {
        this.indexer = indexer;
        this.kicker = kicker;
        this.flywheel = flywheel;
        this.hood = hood;
    }

    public StateMachine build() {
        Supplier<Distance> dummySupplier = () -> Meters.of(0);
        var stateMachine = new StateMachine("Superstructure");

        State TARGETED_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    flywheel.runTargeted(dummySupplier),
                    hood.runTargeted(dummySupplier)
                );

                flywheel.atGoalVelocity().and(hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        kicker.runIntake().alongWith(indexer.runIntake()).named("Superstructure/TargetedShooting/Feeding")
                    );
                
                kicker.isJammed()
                    .onTrue(kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/TargetedShooting")
        );

        State CLOSE_PRESET_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    flywheel.runClosePresetVelocity(),
                    hood.runClosePreset()
                );

                flywheel.atGoalVelocity().and(hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        kicker.runIntake().alongWith(indexer.runIntake()).named("Superstructure/ClosePresetShooting/Feeding")
                    );
                
                kicker.isJammed()
                    .onTrue(kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/ClosePresetShooting")
        );

        State FAR_PRESET_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    flywheel.runFarPresetVelocity(),
                    hood.runStowPreset()
                );

                flywheel.atGoalVelocity().and(hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        kicker.runIntake().alongWith(indexer.runIntake()).named("Superstructure/FarPresetShooting/Feeding")
                    );
                
                kicker.isJammed()
                    .onTrue(kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/FarPresetShooting")
        );

        State STOW = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    flywheel.runIdleVelocity(),
                    hood.runStowPreset()
                );

                coroutine.park();
            }).named("Superstructure/Stow")
        );


        return stateMachine;
    }


}
