package frc.robot.Mechanisms.Shooter.Hood;

import static org.wpilib.units.Units.Radians;

import org.wpilib.units.measure.Angle;

public class HoodConstants {

    public static final Angle MIN_ANGLE = Radians.of(0);
    public static final Angle MAX_ANGLE = Radians.of(0.432);

    public static final Angle STOW_PRESET = Radians.of(0.05);
    public static final Angle CLOSE_PRESET = Radians.of(0.1);
    public static final Angle FAR_PRESET = Radians.of(0.3);

    public static final Angle ANGLE_TOLERANCE = Radians.of(0.01);

}
