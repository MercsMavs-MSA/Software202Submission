// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmSubsystem.*;
import frc.robot.ArmSubsystem.Arm.ArmState;

public class TeleopCommands
{
    private Arm arm;

    public TeleopCommands(Arm arm)
    {
        this.arm = arm;
    }

    public Command armCommand(ArmState state)
    {
        return Commands.runOnce(() -> {arm.setArmState(state);});
    }

    public Command sequentialCommand()
    {
        return Commands.runOnce(() -> {armCommand(ArmState.DEPLOY_OUT);}).andThen(Commands.waitSeconds(1.0)).andThen(Commands.runOnce(() -> {armCommand(ArmState.STOW);}));
    }
}
