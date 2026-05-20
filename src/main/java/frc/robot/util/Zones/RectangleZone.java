package frc.robot.util.Zones;

import java.util.function.Supplier;

import org.opencv.core.Rect;
import org.wpilib.command3.Trigger;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Translation2d;

public class RectangleZone implements Zone {
    private final Rectangle2d rectangle;

    /**
     * Creates a rectangular zone with the given top left and bottom right corners.
     * @param topLeft the top left corner of the rectangle
     * @param bottomRight the bottom right corner of the rectangle
     */
    public RectangleZone(Rectangle2d rectangle) {
        this.rectangle = rectangle;
    }

    /**
     * Returns a Trigger that is active when the given translation is within the rectangle.
     * @param translation the translation to check
     * @return a Trigger that is active when the given translation is within the rectangle
     */
    
    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
        return new Trigger(() -> rectangle.contains(translation.get()));
    }
    
}
