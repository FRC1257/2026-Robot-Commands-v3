package frc.robot.Mechanisms.Intake.IntakePivot;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

import frc.robot.Mechanisms.Intake.IntakePivot.IntakePivotIO.IntakePivotIOInputs;
import frc.robot.util.Units.UnitUtil;

public class IntakePivot extends Mechanism {

    private final IntakePivotIO io;
    private IntakePivotIOInputs inputs = new IntakePivotIOInputs();

    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = null;

    private Angle goalAngle = Radians.zero();

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakePivotConstants.MAX_ANGULAR_VELOCITY.magnitude(),
                IntakePivotConstants.MAX_ANGULAR_ACCELERATION.magnitude()
            )
        );

        Scheduler.getDefault().addPeriodic(() -> {
            io.updateInputs(inputs);
        });
    }

    public Trigger atGoalAngle() {
        return new Trigger(() -> inputs.leaderAngleRads.isNear(goalAngle, IntakePivotConstants.ANGLE_TOLERANCE));
    }

    private NeedsNameBuilderStage runVoltageCommand(Supplier<Voltage> volts) {
        return runRepeatedly(() -> io.setVoltage(volts.get()));
    }

    private NeedsNameBuilderStage runAngleCommand(Supplier<Angle> angle) {
        return run(co -> {
            setpoint = new TrapezoidProfile.State(inputs.leaderAngleRads.magnitude(), inputs.leaderVelocityRadsPerSec.magnitude());
            while (true) {
                goalAngle = UnitUtil.clamp(angle.get(), IntakePivotConstants.MIN_ANGLE, IntakePivotConstants.MAX_ANGLE);
                goal = new TrapezoidProfile.State(goalAngle.magnitude(), 0.0);
                setpoint = profile.calculate(0.02, setpoint, goal);
                io.setAngle(Radians.of(setpoint.position), RadiansPerSecond.of(setpoint.velocity));
                co.yield();
            }
        });
    }

    public Command runIntakeAngle() {
        return runAngleCommand(() -> IntakePivotConstants.INTAKE_ANGLE)
                .named("IntakePivot/IntakeAngle");
    }

    public Command runStowAngle() {
        return runAngleCommand(() -> IntakePivotConstants.STOW_ANGLE)
                .named("IntakePivot/StowAngle");
    }
    
    
}
