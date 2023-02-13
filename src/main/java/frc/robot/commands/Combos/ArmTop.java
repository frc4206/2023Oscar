// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Arm.Elevator.ElevatorTopCommand;
import frc.robot.commands.Arm.Shoulder.ShoulderTopCommand;
import frc.robot.commands.Arm.Wrist.WristTopCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmTop extends SequentialCommandGroup {
  public ArmTop(ArmSubsystem armSubsystem) {
    addCommands(
      new ParallelCommandGroup(
        new ElevatorTopCommand(armSubsystem),
        new ShoulderTopCommand(armSubsystem),
        new WristTopCommand(armSubsystem)
      ).withTimeout(1.5),

      new ClawOutCommand(armSubsystem)
    );
  }
}
