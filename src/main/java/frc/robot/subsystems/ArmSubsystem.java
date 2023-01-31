// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public static CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static RelativeEncoder encoder = armMotor.getEncoder();
  double encoderPosition = encoder.getPosition();

  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
  }

  //Temporary until I find actual positions for specific arm positions
  public void ArmTop(){
    while (encoderPosition < 10){
      armMotor.set(0.5);
    }
  }

  public void ArmMiddle(){
    while (encoderPosition < 5){
      armMotor.set(0.3);
    }
  }

  public void ReturnArm(){
    armMotor.set(-0.5);
  }
  //------------------------------------------------------------------

  @Override
  public void periodic() {}
}
