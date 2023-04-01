// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;


import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;

/** Single Swerve Module:
 * contains a drive motor and a PID controlled turn motor
 */
public class SwerveModule {

    private WPI_TalonSRX DriveMotor;
 

    private WPI_TalonSRX TurnMotor;
    private RelativeEncoder TurnEncoder;    //assuming we are using cancoders - could replace with throughbore encoder
    private CANCoder turnAbsoluteEncoder;
    

    private double setpoint = 0;
    private double goalSpeed = 0;
    private boolean isInverted = false;

    
    public SwerveModule(int driveMotor,int turnMotor, int absoluteEncoder, double kP, double kI, double kD,double kFF, double kIz){
        DriveMotor = new WPI_TalonSRX(driveMotor);
        

        TurnMotor = new WPI_TalonSRX(turnMotor);
        

        turnAbsoluteEncoder = new CANCoder(absoluteEncoder);
        //set PID values
        TurnMotor.config_kP(0, kP);
        TurnMotor.config_kI(0, kI);
        TurnMotor.config_kD(0, kD);
        TurnMotor.config_IntegralZone(0, kIz);
        TurnMotor.config_kF(0, kFF);

        
        
    }
    //set the wheel angle to a value
    public void setAngle(double angle){
        setpoint = angle;
        //get current wheel angle in degrees
        double wheelAngle = TurnMotor.getSelectedSensorPosition() * DriveConstants.ENCODER_TICKS_TO_DEGREES;
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
        //set the setpoint
        TurnMotor.set(ControlMode.Position, targetAngle / DriveConstants.ENCODER_TICKS_TO_DEGREES);
    }
    public void setToEncoder(){
        double currentPos = turnAbsoluteEncoder.getAbsolutePosition() * DriveConstants.ABSOLUTE_ENCODER_TICKS_TO_DEGREES / DriveConstants.ENCODER_TICKS_TO_DEGREES;
        TurnEncoder.setPosition(currentPos);
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
        double angle = Math.toDegrees(Math.atan2(y, x));
        double speed = Math.hypot(x, y);
        driveSpeedSD(angle, orientation, speed);
    }
    //get functions to retrieve position info
    public void driveSpeed(SwerveSpeeds v, double orientation){
        driveSpeedSD(v.angle, orientation, v.speed);
    }
    public SwerveSpeeds getSpeeds(){
        return SwerveSpeeds.SD(TurnEncoder.getPosition(), DriveMotor.getSelectedSensorVelocity());
    }
    public SwerveSpeeds getSetpoints(){
        return SwerveSpeeds.SD(setpoint, goalSpeed);
    }
    public double getDirection(){
        return TurnEncoder.getPosition();
    }
    public double getSpeed(){
        return DriveMotor.getSelectedSensorVelocity();
    }
    public boolean isInverted(){
        return isInverted;
    }
}
