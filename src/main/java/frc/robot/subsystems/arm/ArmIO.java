package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public boolean isMotorConnected = false;

        public double velocityRotPerSec = 0.0;
        public Rotation2d position = Rotation2d.kZero;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPosition(Rotation2d position) {}

    public default Angle getPosition() { return null; }

    public default void stop() {}

    public default void setBrakeMode(boolean enableBrake) {}
}
