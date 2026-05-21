package frc.robot.Mechanisms.Kicker;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public interface KickerIO {
    
    public static class KickerIOInputs {
        public boolean leaderConnected = false;
        public Angle leaderAngleRads = Radians.zero();
        public AngularVelocity leaderVelocityRadsPerSec = RadiansPerSecond.zero();
        public Voltage leaderAppliedVolts = Volts.zero();
        public Current leaderCurrentAmps = Amps.zero();
        public Temperature leaderTemperatureCelsius = Celsius.zero();
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setVoltage(Voltage volts) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void stop() {}
    
}
