// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.led.CANdle;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class DriveSubsystem extends SubsystemBase {
  private final CANdle leds = new CANdle(18);
  private double limitedInput = 0;
  
  private int ledr = 0;
  private int ledg = 255;
  private int ledb = 0;
  // A DriveSubsystem has 4 WPI_TalonSRX motor controllers 
  private final WPI_TalonSRX Motor_FL = new WPI_TalonSRX(9);
  private final WPI_TalonSRX Motor_FR = new WPI_TalonSRX(8);
  private final WPI_TalonSRX Motor_BL = new WPI_TalonSRX(3);
  private final WPI_TalonSRX Motor_BR = new WPI_TalonSRX(7);

  // A DifferentialDrive controls the front left and front right motors in a tank drive format
  private final DifferentialDrive drivebase = new DifferentialDrive(Motor_FR, Motor_FL);

  
  /** Creates a new DriveSubsystem */
  public DriveSubsystem() {
    // the right motors are inverted, so positive speeds correspond to driving forward on both sides 
    Motor_FR.setInverted(true);
    Motor_BR.setInverted(true);

    //the back left and right motors follow identical motions to their corresponding front motor
    Motor_BL.follow(Motor_FL);
    Motor_BR.follow(Motor_FR);
  }
  public CommandBase setColor(int r, int g, int b){
    return runOnce(() -> {ledr = r; ledg = g; ledb = b;});
  }
  /** Drive the robot with an input for drive speed and turning speed. Both range from -1 to 1*/
  public void ArcadeDrive(double driveSpeed,double turnSpeed){
    
    
    turnSpeed *= -1;
    driveSpeed *= -1;

    limitedInput += Math.max(-0.02, Math.min(0.02, driveSpeed - limitedInput));
    int count = (int) (Math.abs(limitedInput) * 60);
    leds.setLEDs(0,0,0);
    leds.setLEDs(ledr,ledg,ledb,0,0,8);
    if(limitedInput > 0){
      leds.setLEDs(0,255,0,0,8,count);
    }
    else{
      leds.setLEDs(255,0,0,0,8,count);
    }
    //if joystick inputs are really small, pretend they are 0 to avoid jitter/drifting (deadbanding)
    if(Math.abs(driveSpeed) < 0.1){ driveSpeed = 0;}
    if(Math.abs(turnSpeed) < 0.1){ turnSpeed = 0;}

    /* scale the speed by 0.5, so full speed on the joystick equals 50% driving speed */
    driveSpeed *= 1;
    turnSpeed *= 0.6;
    //drive at the desired speed
    drivebase.arcadeDrive(driveSpeed,turnSpeed);

    }

    public CommandBase encoderResetCommand() {
      return runOnce(
        () -> {
        Motor_BL.setSelectedSensorPosition(0);
        });
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Back Left Encoder", Motor_BL.getSelectedSensorPosition());
  }

}
