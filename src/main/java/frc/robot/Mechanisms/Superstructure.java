package frc.robot.mechanisms;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.measure.Distance;

import frc.robot.Robot;
import frc.robot.mechanisms.Indexer.Indexer;
import frc.robot.mechanisms.Kicker.Kicker;
import frc.robot.mechanisms.Shooter.Flywheel.Flywheel;
import frc.robot.mechanisms.Shooter.Hood.Hood;

public class Superstructure {

    private final Robot robot;
    private final Pose2d dummyPose = new Pose2d();

    public Superstructure(Robot robot) {
        this.robot = robot;
    }

    public StateMachine build() {
        Supplier<Distance> dummySupplier = () -> Meters.of(0);
        var stateMachine = new StateMachine("Superstructure");

        State TARGETED_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    robot.flywheel.runTargeted(dummySupplier),
                    robot.hood.runTargeted(dummySupplier)
                );

                robot.flywheel.atGoalVelocity().and(robot.hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        robot.kicker.runIntake().alongWith(robot.indexer.runIntake()).named("Superstructure/TargetedShooting/Feeding")
                    );
                
                robot.kicker.isJammed()
                    .onTrue(robot.kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/TargetedShooting")
        );

        State CLOSE_PRESET_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    robot.flywheel.runClosePresetVelocity(),
                    robot.hood.runClosePreset()
                );

                robot.flywheel.atGoalVelocity().and(robot.hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        robot.kicker.runIntake().alongWith(robot.indexer.runIntake()).named("Superstructure/ClosePresetShooting/Feeding")
                    );
                
                robot.kicker.isJammed()
                    .onTrue(robot.kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/ClosePresetShooting")
        );

        State FAR_PRESET_SHOOTING = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    robot.flywheel.runFarPresetVelocity(),
                    robot.hood.runStowPreset()
                );

                robot.flywheel.atGoalVelocity().and(robot.hood.atGoalAngle())
                    .debounce(Seconds.of(0.5))
                    .whileTrue(
                        robot.kicker.runIntake().alongWith(robot.indexer.runIntake()).named("Superstructure/FarPresetShooting/Feeding")
                    );
                
                robot.kicker.isJammed()
                    .onTrue(robot.kicker.runUnjam());
                
                coroutine.park();
            }).named("Superstructure/FarPresetShooting")
        );

        State STOW = stateMachine.addState(
            Command.noRequirements(coroutine -> {
                coroutine.fork(
                    robot.flywheel.runIdleVelocity(),
                    robot.hood.runStowPreset()
                );

                coroutine.park();
            }).named("Superstructure/Stow")
        );

        State IDLE = stateMachine.addState(
            Command.noRequirements(
                coroutine -> {
                    coroutine.fork(
                        robot.indexer.runIdle(),
                        robot.kicker.runIdle(),
                        robot.flywheel.runIdleVelocity(),
                        robot.hood.runTargeted(dummySupplier)
                    );
            }).named("Superstructure/Idle")
        );

        STOW.switchTo(IDLE).when(robot.dummyTrenchZone.contains(() -> dummyPose.getTranslation()).negate());


        stateMachine.switchFromAny().to(STOW).when(robot.dummyTrenchZone.contains(() -> dummyPose.getTranslation()));
        stateMachine.switchFromAny(CLOSE_PRESET_SHOOTING, FAR_PRESET_SHOOTING, IDLE).to(TARGETED_SHOOTING).when(robot.driver.rightBumper());
        stateMachine.switchFromAny(TARGETED_SHOOTING, FAR_PRESET_SHOOTING, IDLE).to(CLOSE_PRESET_SHOOTING).when(robot.driver.leftPaddle1());
        stateMachine.switchFromAny(TARGETED_SHOOTING, CLOSE_PRESET_SHOOTING, IDLE).to(FAR_PRESET_SHOOTING).when(robot.driver.rightPaddle1());

        stateMachine.setInitialState(IDLE);

        return stateMachine;
    }

}
