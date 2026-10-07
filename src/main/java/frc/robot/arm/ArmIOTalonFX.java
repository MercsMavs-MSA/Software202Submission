package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.arm.ArmConstants.PivotGains;
import frc.robot.arm.ArmConstants.PivotHardware;
import frc.robot.arm.ArmConstants.PivotTalonFXConfiguration;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX pivotMotor;
  private final CANcoder armCANcoder = new CANcoder(ArmConstants.kCANcoderCanID);

  private NeutralModeValue currentMode;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocityRotPerSec;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperatureCelsius;

  public ArmIOTalonFX(
      String canbus,
      PivotHardware hardware,
      PivotGains gains,
      PivotTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {
    pivotMotor = new TalonFX(hardware.pivotID(), canbus);

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();

    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted =
        configuration.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
    motorConfiguration.Feedback.RotorToSensorRatio = 1.0;
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfiguration.Feedback.FeedbackRemoteSensorID = armCANcoder.getDeviceID();

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kD = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kD = gains.g();

    motorConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.41;
    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.41;
    
    motorConfiguration.withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ArmConstants.pivotMaxLimit.getRotations())
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ArmConstants.pivotMinLimit.getRotations()));

    position = pivotMotor.getPosition();
    velocityRotPerSec = pivotMotor.getVelocity();
    appliedVolts = pivotMotor.getMotorVoltage();
    supplyCurrentAmps = pivotMotor.getSupplyCurrent();
    statorCurrentAmps = pivotMotor.getStatorCurrent();
    temperatureCelsius = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        position,
        velocityRotPerSec,
        appliedVolts,
        supplyCurrentAmps,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    pivotMotor.optimizeBusUtilization(0.0, 1.0);
    pivotMotor.getConfigurator().apply(motorConfiguration, 1);
  }

  public ArmIOTalonFX(
      PivotHardware hardware,
      PivotGains gains,
      PivotTalonFXConfiguration configuration,
      double statusSignalUpdateFrequency) {
    this("rio", hardware, gains, configuration, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocityRotPerSec,
                appliedVolts,
                supplyCurrentAmps,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius)
            .isOK();

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
    inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    pivotMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void setPosition(Rotation2d position) {
    pivotMotor.setControl(positionControl.withPosition(position.getRotations()));
  }

  @Override
  public void stop() {
    pivotMotor.setControl(new NeutralOut());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    NeutralModeValue newMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (currentMode != newMode) {
      pivotMotor.setNeutralMode(newMode);
      currentMode = newMode;
    }
  }

  @Override
  public Angle getPosition() {
    return pivotMotor.getPosition().getValue();
  }
}