// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

/** Single Swerve Module:
 * contains a drive motor and a PID controlled turn motor
 */
public class SwerveModule {

    private CANSparkMax DriveMotor;
    private RelativeEncoder DriveEncoder;

    private CANSparkMax TurnMotor;
    private RelativeEncoder TurnEncoder;
    //assuming we are using cancoders - could replace with throughbore encoder
    private CANCoder turnAbsoluteEncoder;
    

    private double setpoint = 0;
    private boolean isInverted = false;
    private SparkMaxPIDController pid;

    //option to have modules arranged in a formation besides a square
    //each module gets a set of coordinates that represents its position relative to the robot's center

    private Translation2d position;  
    
    
    public SwerveModule(int driveMotor,int turnMotor, int absoluteEncoder, Translation2d pos){
        DriveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();

        TurnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);
        TurnEncoder = TurnMotor.getEncoder();

        turnAbsoluteEncoder = new CANCoder(absoluteEncoder);

        
        pid = TurnMotor.getPIDController();
        //set PID values
        pid.setP(DriveConstants.kP);
        pid.setI(DriveConstants.kI);
        pid.setD(DriveConstants.kD);
        pid.setFF(DriveConstants.kIz);
        pid.setIZone(DriveConstants.kFF);

        position = pos;
        
    }
  
    public Translation2d calculateIdealSpeeds(double xspeed, double yspeed, double spinspeed){
        double angle = position.getAngle().getDegrees() + 90;
        double ycomp = Math.cos(angle) * spinspeed;
        double xcomp = -Math.sin(angle) * spinspeed;
        return new Translation2d(xspeed + xcomp, yspeed + ycomp);
    }
    

    //set the wheel angle to a value
    public void setAngle(double angle){
        setpoint = angle;
        //get current wheel angle in degrees
        double wheelAngle = TurnEncoder.getPosition() * DriveConstants.ENCODER_TICKS_TO_DEGREES;
        //get distance to nearest equivalent angle to target angle 
        double error = setpoint - wheelAngle;

        //find the closest version of the target angle. error is now guaranteed to be between -180 and 180
        error = (error - 180) % 360 + 180;

        //if we're still more than 90deg away from target angle, then face in the opposite direction and drive backwards
        //now the farthest it ever needs to spin is 90 degrees
        if(error > 90){error -= 180; isInverted = true;}
        else if(error < -90){error += 180; isInverted = true;}
        else{ isInverted = false;}

        double targetAngle = wheelAngle + error;

        pid.setReference(targetAngle / DriveConstants.ENCODER_TICKS_TO_DEGREES, ControlType.kPosition);
    }
    public void setToEncoder(){
        double currentAngle = turnAbsoluteEncoder.getAbsolutePosition() * DriveConstants.ABSOLUTE_TICKS_TO_DEGREES;
        TurnEncoder.setPosition(currentAngle / DriveConstants.ENCODER_TICKS_TO_DEGREES);
    }
    private void setMotorSpeed(double speed){
        if(isInverted){speed *= -1;}
        DriveMotor.set(speed);
    }
    public void driveSpeedSD(double direction, double orientation, double speed){
        //avoid wheels defaulting to 0 degrees when no speed is applied by not setting the angle if the speed is too low
        if(speed > 0.0001){
            setAngle(direction - orientation);
        }
        setMotorSpeed(speed);
        
    }
    public void driveSpeedXY(double x, double y, double orientation){
        double angle = Math.toDegrees(Math.atan2(y, x));
        double speed = Math.hypot(x, y);
        driveSpeedSD(angle, orientation, speed);
    }
    public void driveSpeed(Translation2d v, double orientation){
        driveSpeedSD(v.getAngle().getDegrees(), orientation, v.getNorm());
    }
    public void driveSpeedMeters(SwerveModuleState state, double orientation){
        driveSpeedSD(state.angle.getDegrees() / DriveConstants.MAX_SPEED,orientation,state.speedMetersPerSecond / DriveConstants.MAX_SPEED);
    }
    
    
    public Rotation2d getDirection(){
        return new Rotation2d(Math.toRadians(TurnEncoder.getPosition() * DriveConstants.ENCODER_TICKS_TO_DEGREES));
    }
    public double getSpeed(){
        return DriveEncoder.getVelocity();
    }
    public double getMetersTraveled(){
        return DriveEncoder.getPosition() * DriveConstants.ENCODER_TICKS_TO_DEGREES;
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed() ,getDirection());
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getMetersTraveled(), getDirection());
    }
    public boolean isInverted(){
        return isInverted;
    }
    
    
    public double ScaleSpeed(double maxLength){
        return  position.getNorm() / maxLength;
    }
}
