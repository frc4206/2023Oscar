package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public static SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    double[] ypr = new double[3];
    double currentAngle;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        gyro.getYawPitchRoll(ypr);
        currentAngle = ypr[1];
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    //Charge Station AutoBalancing
    //-------------------------------------------------------------------------------------------------------
  
    public void BalanceBrake(){
        Translation2d translation = new Translation2d(0, 0.05).times(Constants.Swerve.maxSpeed);
        double rotation = 0;
        drive(translation, rotation, true, true);
    }

    public void AutoBalanceClose(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        double kP = 0.08;

        double errorP = Math.abs(pitch) - 8;
        double outputP = errorP*kP;

        double errorR = Math.abs(roll) - 8;
        double outputR = errorR*kP;
        
        if (!(pitch < 2 && pitch > -2)){
            Translation2d translation = new Translation2d(outputP, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 

        if (!(roll < 2 && roll > -2)) {
            Translation2d translation = new Translation2d(outputR, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 
        
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed-2);
        double rotation = 0;
        drive(translation, rotation, true, true);    
    }

    
    public void AutoBalanceFar(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        double kP = 0.08;

        double errorP = Math.abs(pitch) - 8;
        double outputP = errorP*kP;

        double errorR = Math.abs(roll) - 8;
        double outputR = errorR*kP;
        
        if (!(pitch < 2 && pitch > -2)){
            Translation2d translation = new Translation2d(-outputP, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 

        if (!(roll < 2 && roll > -2)) {
            Translation2d translation = new Translation2d(-outputR, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 
        
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed-2);
        double rotation = 0;
        drive(translation, rotation, true, true);    
    }
    //-------------------------------------------------------------------------------------------------------


    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Yaw", ypr[0]);
        SmartDashboard.putNumber("Pitch", ypr[1]);
        SmartDashboard.putNumber("Roll", ypr[2]);

        for (SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}