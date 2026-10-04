package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public enum ArmState {
        STOW,
        DEPLOY_OUT,
        DEPLOY_IN
    }

    public ArmState armState;

    private final ArmIO armIO;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO arm) {
        armIO = arm;
        armState = ArmState.STOW;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm/Inputs", armInputs);

        switch (armState) {
            case STOW:
                armIO.setPosition(Rotation2d.kZero);
            case DEPLOY_OUT:
                armIO.setPosition(Rotation2d.fromRotations(0.25));
            case DEPLOY_IN:
                armIO.setPosition(Rotation2d.fromRotations(-0.25));
            default:
        }
    }

    public void setArmState(ArmState state) {
        armState = state;
    }

    @AutoLogOutput
    public ArmState getArmState() {
        return armState;
    }
}
