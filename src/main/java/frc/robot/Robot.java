package frc.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Translation2d;

import frc.robot.mechanisms.Indexer.Indexer;
import frc.robot.mechanisms.Indexer.IndexerIO;
import frc.robot.mechanisms.Intake.IntakePivot.IntakePivot;
import frc.robot.mechanisms.Intake.IntakePivot.IntakePivotIO;
import frc.robot.mechanisms.Intake.IntakeRollers.IntakeRollers;
import frc.robot.mechanisms.Intake.IntakeRollers.IntakeRollersIO;
import frc.robot.mechanisms.Kicker.Kicker;
import frc.robot.mechanisms.Kicker.KickerIO;
import frc.robot.mechanisms.Shooter.Flywheel.Flywheel;
import frc.robot.mechanisms.Shooter.Flywheel.FlywheelIO;
import frc.robot.mechanisms.Shooter.Hood.Hood;
import frc.robot.mechanisms.Shooter.Hood.HoodIO;
import frc.robot.util.Zones.RectangleZone;
import frc.robot.util.Zones.Zone;

public class Robot extends OpModeRobot {

    public final Flywheel flywheel = new Flywheel(new FlywheelIO() {});
    public final Hood hood = new Hood(new HoodIO() {});
    public final Kicker kicker = new Kicker(new KickerIO() {});
    public final Indexer indexer = new Indexer(new IndexerIO() {});
    public final IntakePivot intakePivot = new IntakePivot(new IntakePivotIO() {});
    public final IntakeRollers intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
    public final CommandGamepad driver = new CommandGamepad(0);

    public final Zone dummyTrenchZone = new RectangleZone(new Rectangle2d(new Translation2d(), new Translation2d()));

    public Robot() {

    }

    @Override
    public void robotPeriodic() {
        Scheduler.getDefault().run();

    }









    
}
