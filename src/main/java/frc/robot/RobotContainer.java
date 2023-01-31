// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmMiddleCommand;
import frc.robot.commands.Arm.ArmTopCommand;
import frc.robot.commands.Arm.ReturnArmCommand;
import frc.robot.commands.Claw.ClawShifterCommand;
import frc.robot.commands.Swerve.AutoBalanceCloseCommand;
import frc.robot.commands.Swerve.AutoBalanceFarCommand;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  private final SendableChooser<String> autoChooser = new SendableChooser<String>();


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


  /* Subsystems */
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();


  /* Event Maps (Autos) */
  public static Map<String, Command> linkEventMap1 = new HashMap<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));


    linkEventMap1.put("arm1", new SequentialCommandGroup(new ArmTopCommand(arm), new ClawShifterCommand(claw), new ReturnArmCommand(arm)));
    linkEventMap1.put("harvest1", new ClawShifterCommand(claw));
    linkEventMap1.put("arm2", new SequentialCommandGroup(new ArmTopCommand(arm), new ClawShifterCommand(claw), new ReturnArmCommand(arm)));
    linkEventMap1.put("harvest2", new ClawShifterCommand(claw));
    linkEventMap1.put("arm3", new SequentialCommandGroup(new ArmTopCommand(arm), new ClawShifterCommand(claw), new ReturnArmCommand(arm)));

    autoChooser.addOption("Link Barrier Side Blue", "Link Barrier Side Blue");
    autoChooser.addOption("Link Barrier Side Red", "Link Barrier Side Red");
    autoChooser.addOption("Link Wall Side Blue", "Link Wall Side Blue");
    autoChooser.addOption("Link Wall Side Red", "Link Wall Side Red");

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    //Swerve Commands
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new AutoBalanceCloseCommand(swerve));
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new AutoBalanceFarCommand(swerve));
  
    //Arm Commands
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).onTrue(new ArmTopCommand(arm));
    new JoystickButton(operator, XboxController.Axis.kLeftTrigger.value).onTrue(new ArmMiddleCommand(arm));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).onTrue(new ReturnArmCommand(arm));
  
    //Claw Commands
    new JoystickButton(operator, XboxController.Button.kX.value).onTrue(new ClawShifterCommand(claw));
  }

  public void setRumble(){
    driver.setRumble(RumbleType.kLeftRumble, 1);
    driver.setRumble(RumbleType.kRightRumble, 1);
  }

  public void offRumble(){
    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);
  }


  public Command getAutonomousCommand() {
    String selectedpath = autoChooser.getSelected();
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(selectedpath, new PathConstraints(4, 3));
    
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerve::getPose, 
        swerve::resetOdometry, 
        Constants.Swerve.swerveKinematics, 
        new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), 
        new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD), 
        swerve::setModuleStates, 
        linkEventMap1,
        true, 
        swerve 
    );
    
    Command fullAuto = autoBuilder.fullAuto(pathGroup);
    return fullAuto;
  }
}