// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Conversions;
import frc.lib.OnboardModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private WPI_TalonFX angleMotor;
    private WPI_TalonFX driveMotor;

    //private RelativeEncoder driveEncoder;
    //private RelativeEncoder integratedAngleEncoder;
  
    private CANCoder angleEncoder;

    //private final SparkMaxPIDController driveController;
   // private final SparkMaxPIDController angleController;

   private final SimpleMotorFeedforward feedforward =
   new SimpleMotorFeedforward(
       Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            //gets meters per second the robot is driving
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
            Constants.SwerveConstants.wheelCircumference,
            Constants.SwerveConstants.driveGearRatio), 
            //gets the angle of the robot
            getAngle()
        ); 
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV supports this now so dont have to worry with rev, have to be sad with falcon
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState){
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //the ? and : are a shorthand for an if-else loop
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; 
        
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            //when not taking feedback
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference,
                                                      Constants.SwerveConstants.driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }



    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.SwerveConstants.angleGearRatio);
        //double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        //integratedAngleEncoder.setPosition(absolutePosition);
        angleMotor.setSelectedSensorPosition(absolutePosition); //might need to change bc of ctre is dumb

      }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }
    
    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        //factory resets motor settings
        angleMotor.configFactoryDefault();
        //configs all settings, feedforward, pid, ramps, etc.
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        //inverts motor if needed
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        //sets brake or not brake
        angleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        //resets angle encoder to match absolute encoder
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        //factory resets motor
        driveMotor.configFactoryDefault();
        //configs all settings, pid, ramps, etc
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        //sets inverted if needed
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        //sets to brake or coast
        driveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        //
        driveMotor.setSelectedSensorPosition(0);
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
    }
    
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            //gets distance the module has moved
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
            Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio), 
            //gets current angle of module
            getAngle()
        );
    }








  

}
