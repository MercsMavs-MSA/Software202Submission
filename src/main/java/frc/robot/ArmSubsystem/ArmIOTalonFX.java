
package frc.robot.subsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre,phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.subsystems.ArmConstants.ArmGains;
import frc.robot.subsystems.ArmConstants.ArmHardware;
import frc.robot.subsystems.ArmConstants.ArmTalonFXConfiguration;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX armMotor;
    private NeutralModeValue currentMode;

    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocityRotPerSec;
    private StatusSignal<Current> supplyCurrentAmps;
    private StatusSignal<Current> statorCurrentAmps;
    private StatusSignal<Voltage> appliedVoltage;
    private StatusSignal<Temperature> temperatureCelsius;

    public ArmIOTalonFX(String canbus, ArmHardware hardware, ArmGains gains, ArmTalonFXConfiguration configuration, double statusSignalUpdateFrequency) 
    {
        armMotor = new TalonFX(hardware.armID(), canbus);
        motorConfiguration.CurrentLimits.SupplyCurentLimitEnable = configuration.enableSupplyCurrentLimit();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = configuration.enableStatorCurrentLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();

        motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
        motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();

        motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
        motorConfiguration.MotorOutput.Inverted = configuration.invert() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwose_Positive;
        motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
        motorConfiguration.Feedback.RotorToSensorRatio = 1.0;

        motorConfiguration.Slot0.kP = gains.p();
        motorConfiguration.Slot0.kI = gains.i();
        motorConfiguration.Slot0.kD = gains.d();
        motorConfiguration.Slot0.kS = gains.s();
        motorConfiguration.Slot0.kV = gains.v();
        motorConfiguration.Slot0.kA = gains.a();
        motorConfiguration.Slot0.kG = gains.g();

        motorConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfiguration.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(ArmConstants.armMaxLimit.getRotations()).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(ArmConstants.armMinLimit.getRotations()));

        position = armMotor.getPosition();
        velocityRotPerSec = armMotor.getVelocity();
        appliedVoltage = armMotor.getMotorVoltage();
        supplyCurrentAmps = armMotor.getSupplyCurrent();
        statorCurrentAmps = armMotor.getStatorCurrent();
        temperatureCelsius = armMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(statusSignalUpdateFrequency, position, velocityRotPerSec, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temperatureCelsius);

        armMotor.optimizeBusUtilization(0.0, 1,0);
        armMotor.getConfigurator().apply(motorConfiguration, 1);

    }

    public ArmIOTalonFX(ArmHardware hardware, ArmGains gains, ArmTalonFXConfiguration configuration, double statusSignalUpdateFrequency)
    {
        this("rio", hardware, gains, configuration, statusSignalUpdateFrequency);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs)
    {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(position, velocityRotPerSec, appliedVoltage, supplyCurrentAmps, statorCurrentAmps, temperatureCelsius).isOK(); 
        //supplyCurrentAmps came twice in worldtour code, is it a mistake?

        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble(); 

        //why do we use getValueAsDouble if velocityRotPerSec and appliedVoltage are 
        //already instantiated as doubles in the IO file?

        //The Intake file uses appliedVoltage and appliedVolts, is that a typo or
        //do you actually use both? the IO file refers to appliedVoltage not appliedVolts.

        //so far, I've changed mine to only use appliedVoltage.

        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();

    }
    
    



}  

    

    

