package frc.robot.Mechanisms.Kicker;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public class KickerConstants {

    public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.05);

    public static final AngularVelocity IDLE_PRESET = RadiansPerSecond.of(0);
    public static final AngularVelocity EJECT_PRESET = RadiansPerSecond.of(-100);
    public static final AngularVelocity INTAKE_PRESET = RadiansPerSecond.of(100);
    public static final AngularVelocity UNJAM_PRESET = RadiansPerSecond.of(-200);

    public static final Current UNJAM_CURRENT_THRESHOLD = Amps.of(60);
}
