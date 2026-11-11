package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ARM_Subsystem.ArmConstants;
import frc.robot.subsystems.ARM_Subsystem.ArmIO;
import frc.robot.subsystems.ARM_Subsystem.ArmIOSim;
import frc.robot.subsystems.ARM_Subsystem.ArmIOTalonFX;

public class RobotContainer {
  private Arm arm;
  private CommandXboxController xboxController = new CommandXboxController(0);
  public RobotContainer() {
    switch(ArmConstants.getMode())
    {
      case REAL:
        arm = new Arm(new ArmIOTalonFX
          (
          ArmConstants.armHardware, ArmConstants.armGains, ArmConstants.kArmConfiguration, ArmConstants.kStatusSignalUpdateFrequencyHz
          )
        );
        break;
      case SIM:
        arm = new Arm(new ArmIOSim(
          0, 
          null,
          null
          )
        );
        break;
      case REPLAY:
        arm = new Arm(
          new ArmIO
            (
              // This is a placeholder for the replay mode. You can implement the replay functionality as needed.
            ) 
              {
                // Implement the necessary methods for replay mode here
              }
        );
        break;
    }

    new TeleopCommands(arm);
    configureBindings();
    
  }

  private void configureBindings() {
    xboxController.leftTrigger().onTrue(
      Commands.runOnce(
        () -> arm.setArmState(Arm.State.DEPLOY_OUT), arm
      )
    );
    xboxController.rightTrigger().onTrue(
      Commands.runOnce(
        () -> arm.setArmState(Arm.State.DEPLOY_IN), arm
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print
    (
        "No autonomous command configured"
    );
  }
}
