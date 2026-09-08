package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmState;

public class TeleopCommands {
    private Arm arm;

    public TeleopCommands(Arm arm) {
        this.arm = arm;
    }

    public Command armCommand(ArmState state) {
        return Commands.runOnce(() -> {
            arm.setState(state);
        }, arm);
    }

    public Command sequentialCommand() {
        return Commands.sequence(
            armCommand(ArmState.DEPLOY_OUT),
            Commands.waitSeconds(1),
            armCommand(ArmState.STOW)
        );
    }
}
