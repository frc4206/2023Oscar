// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test_Auto extends SequentialCommandGroup {
  PathPlannerTrajectory path = PathPlanner.loadPath("Link Barrier Side Blue", new PathConstraints(1.5, 0.75));
  
  public Test_Auto(SwerveSubsystem swerve) {
    var thetaController = new PIDController(
      Constants.AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand group = new PPSwerveControllerCommand(
      path, 
      swerve::getPose, 
      Constants.Swerve.swerveKinematics, 
      new PIDController(0, 0, 0), //x controller
      new PIDController(0, 0, 0), //y controller
      thetaController, 
      swerve::setModuleStates,  
      swerve);
      
    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(path.getInitialHolonomicPose())), 
      group
    );
  }
}
