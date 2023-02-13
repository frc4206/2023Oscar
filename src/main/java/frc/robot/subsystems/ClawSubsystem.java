// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  public static DoubleSolenoid clawPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Claw.forwardChannel, Constants.Claw.reverseChannel);
  
  private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  //private AnalogInput pneumaticPressureSensor = new AnalogInput(Constants.Pneumatics.pneumaticPressureSensor);

  public ClawSubsystem() {
    compressor.enableDigital();
  }

  public void ClawForward(){
    clawPiston.set(Value.kForward);
  }

  public void ClawReverse(){
    clawPiston.set(Value.kReverse);
  }

  public void ClawShifter(){
    switch (clawPiston.get()){
      case kOff:
        ClawForward();
      case kForward:
        ClawReverse();
      case kReverse:
        ClawForward();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
