// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ZoneUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public enum ArmState 
  {
    STOW(() -> Rotation2d.fromRotations(0.0), 0),
    DEPLOY_OUT(() -> Rotation2d.fromRotations(0.0),0.25),
    DEPLOY_IN(() -> Rotation2d.fromRotations(0.0),-0.25);
  }

  public ArmState armState;

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public ArmConstructor(ArmIO armio)
  {
    armIO = armio;
    armState = STOW;
  }

  private Rotation2d armGoal;

  @Override
  public void periodic() 
  {
    armHardware.updateInputs(armInputs);
    Logger.processInputs("Arm/Inputs", armInputs);

    switch (armState)
    {
      case STOW:
        armGoal = armState.getPosition();
        break;
      case DEPLOY_OUT:
        armGoal = armState.getPosition();
        break;
      case DEPLOY_IN:
        armGoal = armState.getPosition();
        break;
    }

    setPosition(armGoal);
  }

  public void setArmState(ArmState state)
  {
    armState = state;
  }

  @AutoLogOutput(key = "Arm/STATE")
  public enum getArmState()
  {
    return armState;
  }


  
}
