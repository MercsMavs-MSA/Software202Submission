package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.ARM_Subsystem.ArmIO.ArmIOInputs;
import frc.robot.subsystems.ARM_Subsystem.ArmIO;

public class Arm extends SubsystemBase {
  public enum State 
  {
    STOW,
    DEPLOY_OUT,
    DEPLOY_IN
  }

  public State armState;
  private final ArmIO armHardware;
  private final ArmIOInputs armInputs = new ArmIOInputs();

  public Arm(ArmIO armIO) 
  {
    armHardware = armIO;
    armState = State.STOW;
  }

  @Override
    public void periodic() 
    {
        armHardware.updateInputs(armInputs);
        Logger.processInputs("Arm/Inputs/Arm", (LoggableInputs) armInputs);

        Logger.recordOutput("Arm/ArmVelocityRotPerSec", armInputs.velocityRotPerSec);
        Logger.recordOutput("Arm/ArmPositionRotations", armInputs.position.getRotations());

        switch (armState) 
        {
            case STOW:
                armHardware.setPosition(Rotation2d.fromRotations(0.0));
                break;
            case DEPLOY_OUT:
                armHardware.setPosition(Rotation2d.fromRotations(0.25));
                break;
            case DEPLOY_IN:
                armHardware.setPosition(Rotation2d.fromRotations(-0.25));
                break;
        }
    }

  public void setArmState(State state) 
  {
    armState = state;
  }
  public void setArmPosition(Rotation2d goal) 
  {
    armHardware.setPosition(goal);
  }

  public void setArmVoltage(double voltage) 
  {
    armHardware.setVoltage(voltage);
  }

  @AutoLogOutput(key = "Arm/POSITIONDEGREES")
  public double getArmPositionDegrees() 
  {
    return armHardware.getPosition().abs(Degrees);
  }

  public void stopArm() 
  {
    armHardware.stop();
  }

  public void setArmBrakeMode(boolean enableBrake) 
  {
    armHardware.setBrakeMode(enableBrake);
  }

  @AutoLogOutput(key = "States/ArmState")
  public State getArmState() 
  {
    return armState;
  }
  
  @AutoLogOutput(key = "Arm/Position")
  public double getArmPosition() 
  {
    return armHardware.getPosition().in(Rotations);
  }

}

