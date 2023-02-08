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
  public static RelativeEncoder elevatorenc;
  public static RelativeEncoder shoulderenc;
  public static RelativeEncoder wristenc;
  public static SparkMaxPIDController epidController;
  public static SparkMaxPIDController spidController;
  public static SparkMaxPIDController wpidController;
  DigitalInput limitswitch;
  double encPositionE = elevatorenc.getPosition();
  double encPositionS = elevatorenc.getPosition();
  double encPositionW = elevatorenc.getPosition();

  public ArmSubsystem() {
  /*----------------------------elevator--------------------------------------------- */
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

/*----------------------------shoulder------------------------------------------------ */
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setInverted(false);
    shoulderMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

    shoulderenc = shoulderMotor.getEncoder();
    spidController = elevatorMotor.getPIDController();
    shoulderenc.setPosition(0.0);
    spidController.setFeedbackDevice(elevatorenc);

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
/*------------------------------wrist-------------------------------------------------- */
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setInverted(false);
    wristMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

    wristenc = wristMotor.getEncoder();
    wpidController = elevatorMotor.getPIDController();
    wristenc.setPosition(0.0);
    wpidController.setFeedbackDevice(elevatorenc);

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
  }

  //Temporary until I find actual positions for specific arm positions
  public void ArmRetrieve(){
    while (encPositionS < 20){
      shoulderMotor.set(0.6);
    }
  }

  public void ArmTop(){
    while (encPositionS < 10){
      shoulderMotor.set(0.5);
    }
  }

  public void ArmMiddle(){
    while (encPositionS < 5){
      shoulderMotor.set(0.3);
    }
  }

  public void ArmLow(){
    while (encPositionS < 1){
      shoulderMotor.set(0.2);
    }
  }

  public void ReturnArm(){
    shoulderMotor.set(-0.5);
  }
  //------------------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", encPositionE);
    SmartDashboard.putNumber("shoulder position", encPositionS);
    SmartDashboard.putNumber("wrist position", encPositionW);
  
  }
}
