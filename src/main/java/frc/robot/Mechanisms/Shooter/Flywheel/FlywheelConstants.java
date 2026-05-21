package frc.robot.Mechanisms.Shooter.Flywheel;

import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.units.measure.AngularVelocity;

public class FlywheelConstants {

    public static final AngularVelocity CLOSE_PRESET = RadiansPerSecond.of(200);
    public static final AngularVelocity FAR_PRESET = RadiansPerSecond.of(300);
    public static final AngularVelocity IDLE_PRESET = RadiansPerSecond.of(50);

    public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(5);
}