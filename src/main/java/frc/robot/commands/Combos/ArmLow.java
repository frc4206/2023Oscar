// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Arm.Elevator.ElevatorLowCommand;
import frc.robot.commands.Arm.Shoulder.ShoulderLowCommand;
import frc.robot.commands.Arm.Wrist.WristLowCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLow extends SequentialCommandGroup {
  public ArmLow(ArmSubsystem armSubsystem) {
    addCommands(
      new ParallelCommandGroup(
        new ElevatorLowCommand(armSubsystem),
        new ShoulderLowCommand(armSubsystem),
        new WristLowCommand(armSubsystem)
      ).withTimeout(1.5),

      new ClawOutCommand(armSubsystem)
    );
  }
}
