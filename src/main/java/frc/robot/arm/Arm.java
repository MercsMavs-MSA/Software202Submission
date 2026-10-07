package frc.robot.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.ArmConstants.PivotHardware;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Arm {
    public enum ArmState {
        STOW,
        DEPLOY_OUT,
        DEPLOY_IN
    }
    
    private ArmState armState;
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    
    public Arm(ArmIO IO) {
        armIO = IO;
        armState = ArmState.STOW;
    }

    public void periodic() {
        PivotHardware.updateInputs(armInputs);
        Logger.processInputs("Arm/Inputs", armInputs);
        Logger.recordOutput("Arm/PivotVelocityRotPerSec", armInputs.velocityRotPerSec);
        switch (armState) {
            case STOW:
                Rotation2d.fromRotations(0.0);
            break;
            case DEPLOY_OUT:
                Rotation2d.fromRotations(0.25);
            break;
            case DEPLOY_IN:
                Rotation2d.fromRotations(-0.25);
            break;
        }
    }

    public void setArmState(ArmState state) {
        armState = state;
    }

    @AutoLogOutput(key = "States/ArmState")
    public ArmState getArmState() {
        return armState;
    }
}
