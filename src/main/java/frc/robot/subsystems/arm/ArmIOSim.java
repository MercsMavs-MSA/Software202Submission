package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.arm.ArmConstants.ArmHardware;
import frc.robot.subsystems.arm.ArmConstants.ArmSimulationConfiguation;

public class ArmIOSim implements ArmIO {
    private final double kLoopPeriodSec;

    private final DCMotorSim armMotor;

    private double appliedVoltage = 0;

    public ArmIOSim(
        double loopPeriodSec,
        ArmHardware hardware,
        ArmSimulationConfiguation config
    ) {
        armMotor =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(
                config.motorType(), config.measurementStdDevs(), hardware.gearing()),
                config.motorType()
            );
        kLoopPeriodSec = loopPeriodSec;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armMotor.update(kLoopPeriodSec);

        inputs.isMotorConnected = true;

        inputs.position = Rotation2d.fromRotations(armMotor.getAngularPositionRotations());
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = 0;
        inputs.statorCurrentAmps = 0;
        inputs.temperatureCelsius = 0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12, 12);
        armMotor.setInputVoltage(appliedVoltage);
    }

    @Override
    public void setPosition(Rotation2d goalPosition) {
        armMotor.setAngle(goalPosition.getRadians());
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    @Override
    public Angle getPosition() {
        return armMotor.getAngularPosition();
    }
}
