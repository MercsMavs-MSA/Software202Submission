package frc.robot.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
  public static final Rotation2d pivotMaxLimit = Rotation2d.fromRotations(0.24);
  public static final Rotation2d pivotMinLimit = Rotation2d.fromRotations(-0.05);
  public static final int kCANcoderCanID = 2;
  
  public record PivotHardware(int pivotID, double gearing) {}

  public record PivotGains(
      double p,
      double i,
      double d,
      double s,
      double v,
      double a,
      double g,
      double maxVelocityRotationsPerSecond,
      double maxAccelerationRotationsPerSecondSquared,
      double jerkRotationsPerSecondCubed) {}

  public record PivotTalonFXConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record ArmSimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}    
  
  public static PivotHardware pivotHardware = new PivotHardware(1, 60);

  public static PivotGains pivotGains =
      new PivotGains(
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

  public static final PivotTalonFXConfiguration kPivotMotorConfiguration =
      new PivotTalonFXConfiguration(
          true,
          true,
          true,
          60.0,
          50.0,
          12.0,
          -12.0,
          NeutralModeValue.Brake);
  
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  public static final ArmSimulationConfiguration pivotSimulationConfiguration =
      new ArmSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);
}