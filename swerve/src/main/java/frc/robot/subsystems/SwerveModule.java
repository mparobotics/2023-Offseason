// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kDrive;

/** Single Swerve Module:
 * contains a drive motor and a PID controlled turn motor
 */
public class SwerveModule {

    private CANSparkMax DriveMotor;
    private RelativeEncoder DriveEncoder;

    private CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    private CANSparkMax TurnMotor;
    private RelativeEncoder TurnEncoder;
    //assuming we are using cancoders - could replace with throughbore encoder
    private CANCoder turnAbsoluteEncoder;
    

    private double setpoint = 0;
    private boolean isInverted = false;
    private SparkMaxPIDController turnPID;
    private SparkMaxPIDController drivePID;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kDrive.kS, kDrive.kV, kDrive.kA);

    //option to have modules arranged in a formation besides a square
    //each module gets a set of coordinates that represents its position relative to the robot's center

    private Translation2d position;  
    
    private void setupDriveMotor(){
        
        DriveMotor.restoreFactoryDefaults();
        DriveMotor.setSmartCurrentLimit(kDrive.DRIVE_CURRENT_LIMIT);
        DriveMotor.setInverted(false);
        DriveMotor.setIdleMode(IdleMode.kBrake);

        DriveMotor.enableVoltageCompensation(kDrive.VOLTAGE_COMPENSATION);

        DriveEncoder = DriveMotor.getEncoder();

        drivePID = DriveMotor.getPIDController();
        //set PID values
        drivePID.setP(kDrive.D_kP);
        drivePID.setI(kDrive.D_kI);
        drivePID.setD(kDrive.D_kD);
        drivePID.setFF(kDrive.D_kIz);
        drivePID.setIZone(kDrive.D_kFF);

        DriveMotor.burnFlash();
    }
    private void setupTurnMotor(){
        TurnMotor.restoreFactoryDefaults();
        TurnMotor.setSmartCurrentLimit(kDrive.TURN_CURRENT_LIMIT);
        TurnMotor.setInverted(false);
        TurnMotor.setIdleMode(IdleMode.kBrake);

        //since we only need the position of the turn motor, we can increase the delay between updates for not essential things like the velocity
        TurnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 500);
        TurnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        TurnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500);
        TurnMotor.enableVoltageCompensation(kDrive.VOLTAGE_COMPENSATION);
        TurnEncoder = TurnMotor.getEncoder();


        turnPID = TurnMotor.getPIDController();
        //set PID values
        turnPID.setP(kDrive.kP);
        turnPID.setI(kDrive.kI);
        turnPID.setD(kDrive.kD);
        turnPID.setFF(kDrive.kIz);
        turnPID.setIZone(kDrive.kFF);

        TurnMotor.burnFlash();
    }
    private void setupAbsEncoder(){
        CANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        CANCoderConfig.sensorDirection = true;
        CANCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
        CANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        turnAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
        turnAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    }
    public SwerveModule(int driveMotor,int turnMotor, int absoluteEncoder, Translation2d pos){
        TurnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);
        DriveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        setupDriveMotor();
        setupTurnMotor();
        setupAbsEncoder();

        

        turnAbsoluteEncoder = new CANCoder(absoluteEncoder);
        
        
        

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
        double wheelAngle = TurnEncoder.getPosition() * kDrive.ENCODER_TICKS_TO_DEGREES;
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

        turnPID.setReference(targetAngle / kDrive.ENCODER_TICKS_TO_DEGREES, ControlType.kPosition);
    }
    public void setToEncoder(){
        double currentAngle = turnAbsoluteEncoder.getAbsolutePosition() * kDrive.ABSOLUTE_TICKS_TO_DEGREES;
        TurnEncoder.setPosition(currentAngle / kDrive.ENCODER_TICKS_TO_DEGREES);
    }
    private void setMotorSpeed(double speed){
        if(isInverted){speed *= -1;}
        DriveMotor.set(speed);
    }
    public void driveSpeedSD(double direction, double orientation, double speed, boolean isOpenLoop){
        //avoid wheels defaulting to 0 degrees when no speed is applied by not setting the angle if the speed is too low
        if(speed > 0.0001){
            setAngle(direction - orientation);
        }
        if(isOpenLoop){
            setMotorSpeed(speed);
        }
        else{
            drivePID.setReference(speed, ControlType.kVelocity, 0, feedforward.calculate(speed));
        }
        
        
    }
    public void driveSpeedXY(double x, double y, double orientation){
        double angle = Math.toDegrees(Math.atan2(y, x));
        double speed = Math.hypot(x, y);
        driveSpeedSD(angle, orientation, speed, true);
    }
    public void driveSpeed(Translation2d v, double orientation){
        driveSpeedSD(v.getAngle().getDegrees(), orientation, v.getNorm(), true);
    }
    public void driveSpeedMeters(SwerveModuleState state, double orientation, boolean isOpenLoopControl){
     
        driveSpeedSD(state.angle.getDegrees() / kDrive.MAX_SPEED,orientation,state.speedMetersPerSecond / kDrive.MAX_SPEED, isOpenLoopControl);
        
    }
    
    
    public Rotation2d getDirection(){
        return new Rotation2d(Math.toRadians(TurnEncoder.getPosition() * kDrive.ENCODER_TICKS_TO_DEGREES));
    }
    public double getSpeed(){
        return DriveEncoder.getVelocity();
    }
    public double getMetersTraveled(){
        return DriveEncoder.getPosition() * kDrive.ENCODER_TICKS_TO_DEGREES;
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
