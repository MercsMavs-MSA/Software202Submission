package frc.robot.commands;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TeleopCommands {
private final Arm arm;

public TeleopCommands(Arm arm) {
		this.arm = arm;
    	}
	public Command stateCommand(Arm.State state) 
	{
		return Commands.runOnce(
		() -> arm.setArmState(state), arm
        );
        
    }
	public Command sequentialCommand() {
		return Commands.sequence(
			Commands.runOnce(
				() -> arm.setArmState(Arm.State.DEPLOY_OUT), arm
	),
			Commands.waitSeconds(1),
			Commands.runOnce(
				() -> arm.setArmState(Arm.State.STOW), arm
			)
            
        );

        
    }
}
