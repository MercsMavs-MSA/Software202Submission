// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public enum ArmState {
        STOW,
        DEPLOY_IN,
        DEPLOY_OUT
    }
    
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    public ArmState armState;

    /** Creates a new ArmSubsystem. */
    public Arm(ArmIO armIO) {
        this.armIO = armIO;
        armState = ArmState.STOW;
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm/Inputs", armInputs);

        switch(armState) {
            case STOW:
                armIO.setPosition(Rotation2d.kZero);
                break;
            case DEPLOY_IN:
                armIO.setPosition(Rotation2d.fromRotations(-0.25));
                break;
            case DEPLOY_OUT:
                armIO.setPosition(Rotation2d.fromRotations(0.25));
        }
    }

    public void setState(ArmState state) {
        armState = state;
    }

    @AutoLogOutput
    public ArmState getState() {
        return armState;
    }
}
