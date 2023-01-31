// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public static CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static DigitalInput topLimSwitch = new DigitalInput(Constants.Arm.topLimSwitchChannel);
  public static DigitalInput midLimSwitch = new DigitalInput(Constants.Arm.midLimSwitchChannel);

  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
  }

  public void ArmTop(){
    while (!topLimSwitch.get()){
      armMotor.set(0.5);
    }
  }

  public void ArmMiddle(){
    while (!midLimSwitch.get()){
      armMotor.set(0.3);
    }
  }

  public void ReturnArm(){
    armMotor.set(-0.5);
  }

  @Override
  public void periodic() {}
}
