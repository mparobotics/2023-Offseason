// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.led.CANdle;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;





public class DriveSubsystem extends SubsystemBase {

 


  // A DriveSubsystem has 4 WPI_TalonSRX motor controllers 
  private final WPI_TalonSRX MotorBR = new WPI_TalonSRX(9);
  private final WPI_TalonSRX MotorBL = new WPI_TalonSRX(8);
  private final WPI_TalonSRX MotorFR = new WPI_TalonSRX(3);
  private final WPI_TalonSRX MotorFL = new WPI_TalonSRX(7);
  
  // A DifferentialDrive controls the front left and front right motors in a tank drive format
  private final DifferentialDrive drivebase = new DifferentialDrive(MotorBL, MotorBR);

  //A Pigeon2 IMU for turning in auto
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(17);


  //a CANdle LED controller for decoration purposes only 
  private final CANdle leds = new CANdle(18);

  //The LED strip has 60 LEDs
  private final int led_count = 60;

  //a version of the controller input fed through a slew rate limiter used for the led strip
  private double limitedInput = 0;

  //an RGB color for the small grid of 8 leds on the CANdle itself;
  private int ledr = 0;
  private int ledg = 255;
  private int ledb = 0;

  /** Creates a new DriveSubsystem */
  public DriveSubsystem() {
    // the right motors are inverted, so positive speeds correspond to driving forward on both sides 
    MotorBL.setInverted(true);
    MotorFL.setInverted(true);

    //the back left and right motors follow identical motions to their corresponding front motor
    MotorBR.follow(MotorFR);
    MotorBL.follow(MotorFL);
  }
  


  /** Drive the robot with an input for drive speed and turning speed. Both range from -1 to 1*/
  public void ArcadeDrive(double driveSpeed,double turnSpeed){
    
    //the "front" block on our drivebase is actually in the back of the robot, so invert controller inputs
    //turnSpeed *= -1;
    //driveSpeed *= -1;



    //apply the slew rate limiter to the driveSpeed. This value is only used to light up the LED strip
    limitedInput += Math.max(-0.02, Math.min(0.02, driveSpeed - limitedInput));
    //convert the drive speed % to a number of LEDs to light up. 0 is all off and 1 and -1 are all on.
    int count = (int) (Math.abs(limitedInput) * led_count);

    //clear leds
    leds.setLEDs(0,0,0);
    //set the small grid of leds (#0 to #7) to the RGB color. 
    leds.setLEDs(ledr,ledg,ledb,0,0,8);
    //if the drive speed is positive, set color to green, otherwise, set color to red
    if(limitedInput > 0){
      leds.setLEDs(0,255,0,0,8,count);
    }
    else{
      leds.setLEDs(255,0,0,0,8,count);
    }



    //if joystick inputs are really small, pretend they are 0 to avoid jitter/drifting aka deadbanding
    if(Math.abs(driveSpeed) < 0.1){ driveSpeed = 0;}
    if(Math.abs(turnSpeed) < 0.1){ turnSpeed = 0;}

    //drive speed is 100%, turn speed is 60%
    driveSpeed *= 1;
    turnSpeed *= 0.6;

    //drive at the desired speed
    drivebase.arcadeDrive(driveSpeed,turnSpeed);
  }

  /** Drive the robot with a given left motor speed and right motor speed (Use in AUTO only) */
  public void TankDrive(double left, double right){
    drivebase.tankDrive(left, right);
  }

  
  //this command sets the LEDs to a specific color
  public CommandBase setColor(int r, int g, int b){
    return runOnce(() -> {ledr = r; ledg = g; ledb = b;});
  }

  /** Get the distance traveled in meters measured by the encoder. */
  public double getEncoderPosition(){
    return MotorFL.getSelectedSensorPosition() * DriveConstants.ENCODER_TO_METERS;
  }
  /** set the encoder position to a position in meters */
  public void setEncoderPosition(double position){
    MotorFL.setSelectedSensorPosition(position / DriveConstants.ENCODER_TO_METERS);
  }
  /** Get the direction of the robot in degrees from the Pigeon2 */
  public double getAngle(){
    return gyro.getAngle();
  }
  /** Set the direction of the robot to an angle in degrees */
  public void setAngle(double angle){
    gyro.setYaw(angle);
  }
  


  



  @Override
  public void periodic(){
    //diplay encoder and gyro values to SmartDashboard
    SmartDashboard.putNumber("Raw Encoder Distance", MotorFL.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Distance", MotorFL.getSelectedSensorPosition() * DriveConstants.ENCODER_TO_METERS);

    SmartDashboard.putNumber("Direction", gyro.getAngle());
  }
}
