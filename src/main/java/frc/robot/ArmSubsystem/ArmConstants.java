// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems;

import org.wpilib.framework.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class ArmConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public record ArmTalonFXConfiguration(
    boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode) {}

  public static final ArmTalonFXConfiguration kArmMotorConfiguration =
    new ArmTalonFXConfiguration(
      true,
      true,
      true,
      60,
      50,
      12,
      -12,
      NeutralModeValue.break);

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
    double jerkRotationsPerSecondCubed) {}

  public static ArmGains armGains =
    new ArmGains(
      60,
      0,
      0,
      0.5,
      0,
      0,
      0,
      0,
      0,
      0);

  public record ArmHardware(int armID, double gearing) {}
  public static ArmHardware armHardware = new ArmHardware(1, 60d / 1d);

  public record IntakeSimulationCOnfiguration(DCMotor motorType, double measurementStdDevs) {}
  public static final ArmSimulationConfiguration armSimulationConfiguration = 
    new ArmSimulationConfiguration(DCMotor.getKrakenX60(1), 0.002);
   
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
    
}