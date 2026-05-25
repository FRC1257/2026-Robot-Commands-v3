package frc.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Translation2d;


public class Robot extends OpModeRobot {
    public final Trigger trigger = new Trigger(() -> true);

    public Robot() {
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getDefault().run();
    }









    
}
