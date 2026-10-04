// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ArmSubsystem;

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
import frc.robot.ArmSubsystem.ArmConstants.ArmHardware;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public enum ArmState 
  {
    STOW(() -> Rotation2d.fromRotations(0)),
    DEPLOY_OUT(() -> Rotation2d.fromRotations(0.25)),
    DEPLOY_IN(() -> Rotation2d.fromRotations(-0.25));

    private Supplier<Rotation2d> armgoal;
    private ArmState(Supplier<Rotation2d> goal)
    {
      armgoal = goal;
    }

    public Rotation2d getPosition()
    {
      return armgoal.get();
    }

  }

  public ArmState armState;

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public void ArmConstructor(ArmIO armio)
  {
    armIO = armio;
    armState = ArmState.STOW;
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

  private final ArmIO ArmHardware;

  public void setArmState(ArmState state)
  {
    armState = state;
  }

 @AutoLogOutput(key = "Arm/STATE")
  public enum getArmState
  {
    armState
  }


  public void setPosition(Rotation2d goal)
  {
    ArmHardware.setPosition(goal);
  }
}
