package frc.robot.Mechanisms.Intake.IntakePivot;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class IntakePivotConstants {
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(3.0);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(8.0);

    public static final Angle MIN_ANGLE = Radians.of(0.0);
    public static final Angle MAX_ANGLE = Radians.of(1.6);

    public static final Angle INTAKE_ANGLE = Radians.of(1.6);
    public static final Angle STOW_ANGLE = Radians.of(0.0);

    public static final Angle ANGLE_TOLERANCE = Radians.of(0.05);
}
