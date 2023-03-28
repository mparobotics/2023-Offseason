// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


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
 
    private double setpoint = 0;

    private boolean isInverted = false;
    private SparkMaxPIDController pid;
    
    public SwerveModule(int driveMotor,int turnMotor, double kP, double kI, double kD,double kFF, double kIz){
        DriveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();

        TurnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);
        TurnEncoder = TurnMotor.getEncoder();
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
        if(error > 90){error -= 180; isInverted = !isInverted;}
        if(error < -90){error += 180; isInverted = !isInverted;}
        double targetAngle = wheelAngle + error;
        pid.setReference(targetAngle, ControlType.kPosition);
    }
    public void driveSpeedSD(double direction, double orientation, double speed){
        setAngle(direction - orientation);
        if(isInverted){speed *= -1;}
        DriveMotor.set(speed);
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
    public double getDirection(){
        return TurnEncoder.getPosition();
    }
    public double getSpeed(){
        return DriveEncoder.getVelocity();
    }
}
