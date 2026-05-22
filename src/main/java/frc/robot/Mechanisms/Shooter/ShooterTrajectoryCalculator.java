package frc.robot.mechanisms.Shooter;

import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

import frc.robot.util.Units.UnitUtil;

public class ShooterTrajectoryCalculator {

    private static ShooterTrajectoryCalculator instance;

    public static ShooterTrajectoryCalculator getInstance() {
        if (instance == null) instance = new ShooterTrajectoryCalculator();
        return instance;
    }

    public record ShooterTrajectoryParameters(
        AngularVelocity flywheelVelocity,
        Angle hoodAngle
    ) {}

    private static final InterpolatingTreeMap<Distance, AngularVelocity> flywheelMap = 
        new InterpolatingTreeMap<>(UnitUtil::inverseInterpolate, UnitUtil::interpolate);
    private static final InterpolatingTreeMap<Distance, Angle> hoodMap =
        new InterpolatingTreeMap<>(UnitUtil::inverseInterpolate, UnitUtil::interpolate);

    public ShooterTrajectoryParameters getParameters(Distance distance) {
        return new ShooterTrajectoryParameters(
            flywheelMap.get(distance),
            hoodMap.get(distance)
        );
    }

    
}
