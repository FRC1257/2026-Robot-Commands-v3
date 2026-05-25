package frc.robot.mechanisms;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Shooter extends Mechanism {

    public Shooter() {

    }

    public Command runDisabled() {
        return run(coroutine -> {
            System.out.println("Running shooter disabled");
        }).named("DISABLED");
    }

    public Command runIdle() {
        return run(coroutine -> {
            System.out.println("Running shooter idle");
        }).named("IDLE");
    }

    public Command runShooting() {
        return run(coroutine -> {
            System.out.println("Running shooter shooting");
        }).named("SHOOTING");
    }

    public Command runPassing() {
        return run(coroutine -> {
            System.out.println("Running shooter passing");
        }).named("PASSING");
    }

    
    
}
