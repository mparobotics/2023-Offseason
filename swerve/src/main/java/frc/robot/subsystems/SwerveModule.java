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
    private double goalSpeed = 0;
    private boolean isInverted = false;
    private SparkMaxPIDController pid;
    
    public SwerveModule(int driveMotor,int turnMotor, int absoluteEncoder, double kP, double kI, double kD,double kFF, double kIz){
        DriveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();

        TurnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);
        TurnEncoder = TurnMotor.getEncoder();

        turnAbsoluteEncoder = new CANCoder(absoluteEncoder);

        pid = TurnMotor.getPIDController();
        //set PID values
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setFF(kFF);
        pid.setIZone(kIz);
        
    }
    //set the wheel angle to a value
    public void setAngle(double angle){
        setpoint = angle;
        //get current wheel angle in degrees
        double wheelAngle = TurnEncoder.getPosition() * DriveConstants.ENCODER_ROTATIONS_TO_DEGREES;
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

        pid.setReference(targetAngle / DriveConstants.ENCODER_ROTATIONS_TO_DEGREES, ControlType.kPosition);
    }
    public void setToEncoder(){
        double currentAngle = turnAbsoluteEncoder.getAbsolutePosition();
        TurnEncoder.setPosition(currentAngle);
    }
    public void driveSpeedSD(double direction, double orientation, double speed){
        //avoid wheels defaulting to 0 degrees when no speed is applied by not setting the angle if the speed is too low
        if(speed > 0.0001){
            setAngle(direction - orientation);
        }
            
        if(isInverted){speed *= -1;}
        DriveMotor.set(speed);
        goalSpeed = speed;
    }
    public void driveSpeedXY(double x, double y, double orientation){
        double angle = Math.atan2(y, x);
        double speed = Math.hypot(x, y);
        driveSpeedSD(angle, orientation, speed);
    }
    //get functions to retrieve position info
    public void driveSpeed(SwerveSpeeds v, double orientation){
        driveSpeedSD(v.angle, orientation, v.speed);
    }
    public SwerveSpeeds getSpeeds(){
        return SwerveSpeeds.SD(TurnEncoder.getPosition(), DriveEncoder.getVelocity());
    }
    public SwerveSpeeds getSetpoints(){
        return SwerveSpeeds.SD(setpoint, goalSpeed);
    }
    public double getDirection(){
        return TurnEncoder.getPosition();
    }
    public double getSpeed(){
        return DriveEncoder.getVelocity();
    }
    public boolean isInverted(){
        return isInverted;
    }
}
