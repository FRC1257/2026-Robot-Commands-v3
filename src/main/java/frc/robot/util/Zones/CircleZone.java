package frc.robot.util.Zones;

import java.util.function.Supplier;

import org.wpilib.command3.Trigger;
import org.wpilib.math.geometry.Translation2d;

public class CircleZone implements Zone {
    
    private final Translation2d center;
    private final double radius;

    /**
     * Creates a circular zone with the given center and radius.
     * @param center the center of the circle
     * @param radius the radius of the circle
     */

    public CircleZone(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    /**
     * Returns a Trigger that is active when the given translation is within the circle.
     * @param translation the translation to check
     * @return a Trigger that is active when the given translation is within the circle
     */

    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
        return new Trigger(() -> translation.get().getDistance(center) <= radius);
    }
    
}
