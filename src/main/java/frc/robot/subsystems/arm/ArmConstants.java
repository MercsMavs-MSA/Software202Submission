package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
    public record ArmTalonFXConfiguration(
        boolean invert,
        boolean enableStatorCurrentLimit,
        boolean enableSupplyCurrentLimit,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        double peakForwardVoltage,
        double peakReverseVoltage,
        NeutralModeValue neutralMode
    ) {}

    public record ArmSimulationConfiguation(
        DCMotor motorType,
        double measurementStdDevs
    ) {}

    public record ArmHardware(int armID, double gearing) {}

    public record ArmGains(
        double p,
        double i,
        double d,
        double s,
        double v,
        double a,
        double g,
        double maxVelocityRotationsPerSecond,
        double maxAccelerationRotationsPerSecondSquared,
        double jerkRotationsPerSecondCubed
    ) {}

    public static final ArmTalonFXConfiguration kArmMotorConfiguation = new ArmTalonFXConfiguration(
        true,
        true,
        true,
        60,
        50,
        12,
        -12,
        NeutralModeValue.Brake
    );

    public static final ArmSimulationConfiguation armSimulationConfiguration = new ArmSimulationConfiguation(
        DCMotor.getKrakenX60(1),
        0.002
    );

    public static ArmHardware armHardware = new ArmHardware(
        1,
        60
    );

    public static ArmGains armGains = new ArmGains(
        60, 
        0, 
        0, 
        0.5,
        0, 
        0,
        0,
        0,
        0,
        0
    );

    public static final double kStatusSignalUpdateFrequencyHz = 100;
}
