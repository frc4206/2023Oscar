// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class ArmSubsystem extends SubsystemBase {
  public static CANSparkMax elevatorMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static CANSparkMax wristMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static CANSparkMax clawMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);
  public static RelativeEncoder elevatorenc;
  public static RelativeEncoder shoulderenc;
  public static RelativeEncoder wristenc;
  public static RelativeEncoder clawenc;
  public static SparkMaxPIDController epidController;
  public static SparkMaxPIDController spidController;
  public static SparkMaxPIDController wpidController;
  public static SparkMaxPIDController cpidController;
  DigitalInput limitswitch;
  double encPositionE = elevatorenc.getPosition();
  double encPositionS = elevatorenc.getPosition();
  double encPositionW = elevatorenc.getPosition();
  double encPositionC = clawenc.getPosition();

  public ArmSubsystem() {
    /*----------------------------elevator---------------------------------------------*/
      elevatorMotor.restoreFactoryDefaults();
      elevatorMotor.setIdleMode(IdleMode.kBrake);
      elevatorMotor.setInverted(false);
      elevatorMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec


      elevatorenc = elevatorMotor.getEncoder();
      epidController = elevatorMotor.getPIDController();
      elevatorenc.setPosition(0.0);
      epidController.setFeedbackDevice(elevatorenc);

      epidController.setP(5e-5);
      epidController.setI(1e-6);
      epidController.setD(0);
      epidController.setIZone(0);
      epidController.setFF(.000156);
      epidController.setOutputRange(-0.5, 0.5);
      epidController.setSmartMotionMaxVelocity(2000, 0);
      epidController.setSmartMotionMinOutputVelocity(-2000, 0);
      epidController.setSmartMotionMaxAccel(100, 0);
      epidController.setSmartMotionAllowedClosedLoopError(1, 0);

    /*----------------------------shoulder---------------------------------------------*/
      shoulderMotor.restoreFactoryDefaults();
      shoulderMotor.setIdleMode(IdleMode.kBrake);
      shoulderMotor.setInverted(false);
      shoulderMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

      shoulderenc = shoulderMotor.getEncoder();
      spidController = shoulderMotor.getPIDController();
      shoulderenc.setPosition(0.0);
      spidController.setFeedbackDevice(shoulderenc);

      spidController.setP(5e-5);
      spidController.setI(1e-6);
      spidController.setD(0);
      spidController.setIZone(0);
      spidController.setFF(.000156);
      spidController.setOutputRange(-0.5, 0.5);
      spidController.setSmartMotionMaxVelocity(2000, 0);
      spidController.setSmartMotionMinOutputVelocity(-2000, 0);
      spidController.setSmartMotionMaxAccel(100, 0);
      spidController.setSmartMotionAllowedClosedLoopError(1, 0);

    /*------------------------------wrist----------------------------------------------*/
      wristMotor.restoreFactoryDefaults();
      wristMotor.setIdleMode(IdleMode.kBrake);
      wristMotor.setInverted(false);
      wristMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

      wristenc = wristMotor.getEncoder();
      wpidController = wristMotor.getPIDController();
      wristenc.setPosition(0.0);
      wpidController.setFeedbackDevice(wristenc);

      wpidController.setP(5e-5);
      wpidController.setI(1e-6);
      wpidController.setD(0);
      wpidController.setIZone(0);
      wpidController.setFF(.000156);
      wpidController.setOutputRange(-0.5, 0.5);
      wpidController.setSmartMotionMaxVelocity(2000, 0);
      wpidController.setSmartMotionMinOutputVelocity(-2000, 0);
      wpidController.setSmartMotionMaxAccel(100, 0);
      wpidController.setSmartMotionAllowedClosedLoopError(1, 0);

    /*------------------------------claw-----------------------------------------------*/
      clawMotor.restoreFactoryDefaults();
      clawMotor.setIdleMode(IdleMode.kBrake);
      clawMotor.setInverted(false);
      clawMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec
      
      clawenc = clawMotor.getEncoder();
      cpidController = clawMotor.getPIDController();
      clawenc.setPosition(0.0);
      cpidController.setFeedbackDevice(clawenc);
      
      cpidController.setP(5e-5);
      cpidController.setI(1e-6);
      cpidController.setD(0);
      cpidController.setIZone(0);
      cpidController.setFF(.000156);
      cpidController.setOutputRange(-0.5, 0.5);
      cpidController.setSmartMotionMaxVelocity(2000, 0);
      cpidController.setSmartMotionMinOutputVelocity(-2000, 0);
      cpidController.setSmartMotionMaxAccel(100, 0);
      cpidController.setSmartMotionAllowedClosedLoopError(1, 0);
  }

  //Temporary until I find actual positions for specific arm positions
  public void ElevatorRetrieve(){
    while (encPositionE < 20){
      elevatorMotor.set(0.6);
    }
  }

  public void ElevatorTop(){
    while (encPositionE < 10){
      elevatorMotor.set(0.5);
    }
  }

  public void ElevatorMiddle(){
    while (encPositionE < 5){
      elevatorMotor.set(0.3);
    }
  }

  public void ElevatorLow(){
    while (encPositionE < 1){
      elevatorMotor.set(0.2);
    }
  }

  public void ReturnElevator(){
    while (encPositionE != 0){
      elevatorMotor.set(-0.5);
    }
  }


  
  public void ShoulderRetrieve(){
    while (encPositionS < 20){
      shoulderMotor.set(0.6);
    }
  }

  public void ShoulderTop(){
    while (encPositionS < 10){
      shoulderMotor.set(0.5);
    }
  }

  public void ShoulderMiddle(){
    while (encPositionS < 5){
      shoulderMotor.set(0.3);
    }
  }

  public void ShoulderLow(){
    while (encPositionS < 1){
      shoulderMotor.set(0.2);
    }
  }

  public void ReturnShoulder(){
    while (encPositionS != 0){
      shoulderMotor.set(-0.5);
    }
  }



  public void WristRetrieve(){
    while (encPositionW < 20){
      wristMotor.set(0.6);
    }
  }

  public void WristTop(){
    while (encPositionW < 10){
      wristMotor.set(0.5);
    }
  }

  public void WristMiddle(){
    while (encPositionW < 5){
      wristMotor.set(0.3);
    }
  }

  public void WristLow(){
    while (encPositionW < 1){
      wristMotor.set(0.2);
    }
  }

  public void ReturnWrist(){
    while (encPositionW != 0){
      wristMotor.set(-0.5);
    }
  }


  public void ClawIn(){
    clawMotor.set(0.5);
  }

  public void ClawOut(){
    clawMotor.set(-0.5);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", encPositionE);
    SmartDashboard.putNumber("shoulder position", encPositionS);
    SmartDashboard.putNumber("wrist position", encPositionW);
  }
}
