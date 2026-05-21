package frc.robot.Mechanisms.Kicker;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;

public class KickerConstants {

    public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.05);

    public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(0);
    public static final AngularVelocity EJECT_VELOCITY = RadiansPerSecond.of(-100);
    public static final AngularVelocity INTAKE_VELOCITY = RadiansPerSecond.of(100);
    public static final AngularVelocity UNJAM_VELOCITY = RadiansPerSecond.of(-200);


    public static final AngularVelocity KICKER_JAMMED_VELOCITY = RadiansPerSecond.of(10);
    public static final Current KICKER_JAMMED_CURRENT = Amps.of(20);
    public static final Time KICKER_JAMMED_TIME = Seconds.of(0.5);
}
