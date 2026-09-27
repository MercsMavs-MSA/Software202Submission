// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmSubsystem;

public class TeleopCommands
{
    private ArmSubsystem arm;

    public TeleopCommands(ArmSubsystem arm)
    {
        this.arm = arm;
    }

    public Command armCommand(ArmState state)
    {
        return Commands.runOnce(()-> {arm.setArmState(state);});
    }

    public Command sequentialCommand()
    {
        return Commands.runOnce(() -> arm.DEPLOY_OUT).andThen(Commands.waitSeconds(1.0)).andThen(Commands.runOnce(()-> arm.STOW));
    }
}
