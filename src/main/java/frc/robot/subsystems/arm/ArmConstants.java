package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
    public record ArmHardware(int id, double gearing) {
    }

    public record ArmGains(
            double p,
            double i,
            double d,
            double s,
            double v,
            double a,
            double g,
            double maxVelocityRotationsPerSecond,
            double maxAccelerationRotationsPerSecondSquared) {
    }

    public record ArmTalonFXConfiguration(
            boolean invert,
            boolean enableStatorCurrentLimit,
            boolean enableSupplyCurrentLimit,
            double statorCurrentLimitAmps,
            double supplyCurrentLimitAmps,
            double peakForwardVoltage,
            double peakReverseVoltage,
            NeutralModeValue neutralMode) {
    }

    public record ArmSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {
    }

    public static ArmHardware armHardware = new ArmHardware(1, 60);

    public static ArmGains armGains = new ArmGains(60, 0, 0, 0.5, 0, 0, 0, 1000000, 100000000);

    public static ArmTalonFXConfiguration armTalonFXConfiguration = new ArmTalonFXConfiguration(
        true, 
        true,
        true,
        60,
        50,
        12, 
        -12,
        NeutralModeValue.Brake);

    public static ArmSimulationConfiguration armSimulationConfiguration = 
        new ArmSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);

    public static final double statusSignalUpdateFrequency = 100;
}
