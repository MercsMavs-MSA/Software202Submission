package frc.robot.subsystems.ARM_Subsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
  public static final Rotation2d armMaxLimit = Rotation2d.fromRotations(0.24);
  public static final Rotation2d armMinLimit = Rotation2d.fromRotations(-0.05);
    private static final Mode current_Mode = Mode.REAL;
  
  
 public record ArmHardware(int armID, double gearing) {}
  
    public record ArmGains(
        double P,
        double S) {}
  
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
    public static Mode getMode() {
        return current_Mode;
    }
  
    public enum Mode {
        REAL,
        SIM,
        REPLAY;
    }
    public record ArmSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}
  
    public static ArmHardware armHardware = new ArmHardware(1, 60);
  
    public static ArmGains armGains =
        new ArmGains(
            60, // p
            0.5 // s
            );
  
    public static final ArmTalonFXConfiguration kArmConfiguration =
        new ArmTalonFXConfiguration(
            true, // Invert
            true, // Enable stator current limiting
            true, // Enable supply current limiting
            60.0, // Stator limit
            50.0, // Supply limit
            12.0, // Peak forward voltage
            -12.0, // Peak reverse voltage
            NeutralModeValue.Brake); // Idle mode


  public static final ArmSimulationConfiguration armSimulationConfiguration =
      new ArmSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
}