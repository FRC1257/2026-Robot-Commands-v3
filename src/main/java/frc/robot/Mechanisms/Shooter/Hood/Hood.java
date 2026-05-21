package frc.robot.Mechanisms.Shooter.Hood;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

import frc.robot.Mechanisms.Shooter.ShooterTrajectoryCalculator;
import frc.robot.Mechanisms.Shooter.Hood.HoodIO.HoodIOInputs;
import frc.robot.util.Units.UnitUtil;

public class Hood extends Mechanism {

    private final HoodIO io;
    private HoodIOInputs inputs = new HoodIOInputs();

    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = null;

    private Angle goalAngle = Radians.zero();

    public Hood(HoodIO io) {

        this.io = io;

        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0, 0)
        );

        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    public Trigger atGoalAngle() {
        return new Trigger(() -> inputs.leaderAngleRads.isNear(goalAngle, HoodConstants.ANGLE_TOLERANCE));
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()))
                .whenCanceled(io::stop);
    }

    private NeedsNameBuilderStage runAngleCommand(Supplier<Angle> angle) {
        return run(co -> {
            setpoint = new TrapezoidProfile.State(inputs.leaderAngleRads.magnitude(), inputs.leaderVelocityRadsPerSec.magnitude());
            while (true) {
                goalAngle = UnitUtil.clamp(angle.get(), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
                goal = new TrapezoidProfile.State(goalAngle.magnitude(), 0.0);
                setpoint = profile.calculate(0.02, setpoint, goal);
                io.setAngle(Radians.of(setpoint.position), RadiansPerSecond.of(setpoint.velocity));
                co.yield();
            }
        });
    }

    public Command runTargeted(Supplier<Distance> targetDistance) {
        return runAngleCommand(() -> ShooterTrajectoryCalculator.getInstance().getParameters(targetDistance.get()).hoodAngle())
                .named("Hood/Targeted");
    }

    public Command runStowPreset() {
        return runAngleCommand(() -> HoodConstants.STOW_PRESET)
                .named("Hood/StowPreset");
    }

    public Command runClosePreset() {
        return runAngleCommand(() -> HoodConstants.CLOSE_PRESET)
                .named("Hood/ClosePreset");
    }

    public Command runFarPreset() {
        return runAngleCommand(() -> HoodConstants.FAR_PRESET)
                .named("Hood/FarPreset");
    }

}
