// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;





import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;



public class DriveSubsystem extends SubsystemBase {
  private RobotContainer m_robotContainer;
  
  //drive base motors
  private final CANSparkMax motorFR = new CANSparkMax(DriveConstants.MOTOR_FR_ID, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(DriveConstants.MOTOR_FL_ID, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(DriveConstants.MOTOR_BR_ID, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(DriveConstants.MOTOR_BL_ID, MotorType.kBrushless);
  //encoders to measure driving speed
  //public final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  //public final int kCPR = 8192;
  public RelativeEncoder encoderL = motorFL.getEncoder();
  public RelativeEncoder encoderR = motorFR.getEncoder();
  
  
  //setting ramp

  //create motor controller groups
  private final MotorControllerGroup SCG_R = new MotorControllerGroup(motorFR, motorBR);
  private final MotorControllerGroup SCG_L = new MotorControllerGroup(motorFL, motorBL);

  //differential drive to control the motors
  private final DifferentialDrive differentialDrive = new DifferentialDrive(motorFL, motorFR);
  // error for driving straight
  public double error;

  //slew rate limiters adjust the ammount an input can change by in one second
  //for example, a slew rate limite of .5 would only let the joystick value
  //change by .5 over a second, making slowdowns more gradual
  //this is useful in preventing tippy robot syndrome
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1.7);
  //solenoid to control gear shifting
private DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 6              );
  public Boolean inHighGear = false;
  
  public Boolean BrakeMode = true;
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.PIGEON_ID);
  
  //odometry object 
  private final DifferentialDriveOdometry m_odometry;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    //back motors follow front motors
    motorFR.setOpenLoopRampRate(0); // X seconds from neutral to full output (during open-loop control)
    //falconFR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
  
    motorFL.setOpenLoopRampRate(0); // X seconds from neutral to full output (during open-loop control)
    //falconFL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
  
    motorBL.setOpenLoopRampRate(0); // X seconds from neutral to full output (during open-loop control)
    //falconBL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
  
    motorBR.setOpenLoopRampRate(0); // X seconds from neutral to full output (during open-loop control)
    //falconBR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)
    

  
  
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);
    //invert left motors
    motorFL.setInverted(true);
    motorBL.setInverted(true);
    //dont invert right motors
    motorFR.setInverted(false);
    motorBR.setInverted(false);
  

    motorFR.setSmartCurrentLimit(30, 60);
    motorBR.setSmartCurrentLimit(30, 60);
    motorFL.setSmartCurrentLimit(30, 60);
    motorBL.setSmartCurrentLimit(30, 60);
    //turn on compressor

  

    //create odometry object using motor positions
    //drive position = motor rotations * rotations to meters conversion constant
    m_odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), 
    encoderL.getPosition() * DriveConstants.ROTATIONS_TO_METERS, 
    encoderR.getPosition() * DriveConstants.ROTATIONS_TO_METERS);
    
  }
  /** shifts the gearbox into high gear */
  public void upShift(){
    //shift into high gear by extending both solenoids
    shiftSolenoid.set(Value.kForward);
    inHighGear = true;
    setCoast();

  }
  /** shifts the gearbox into low gear */
  public void downShift(){
    //shift into low gear by retracting both solenoids
    //only shifts if the magnitude of either motor velocity is less than 3,000
    if ((Math.abs(encoderL.getVelocity()) <= 3000) || (Math.abs(encoderR.getVelocity()) <= 3000)){
    shiftSolenoid.set(Value.kReverse);
    inHighGear = false;
  }
    

  }

  // sets the driving speed of the robot 
  public void setDriveSpeedArcade(double forwardSpeed, double turnSpeed){
    
    //set the driving speed based on a forward speed and turning speed - controlled in ArcadeDrive.java
    if (Math.abs(forwardSpeed) < .1) {forwardSpeed = 0;}//deadzones
    if (Math.abs(turnSpeed) < .1) {turnSpeed = 0;}//deadzones
    if (turnSpeed == 0){
      //driveStraight(forwardSpeed);
      differentialDrive.arcadeDrive(slewRateLimiter.calculate(forwardSpeed) * DriveConstants.DRIVE_SPEED, 0);
    }
    else{
      if ((Math.abs(encoderL.getVelocity()) >= DriveConstants.MAX_DRIVE_SPEED) || (Math.abs(encoderR.getVelocity()) >= DriveConstants.MAX_DRIVE_SPEED))
        {
        differentialDrive.arcadeDrive(slewRateLimiter.calculate(forwardSpeed) * DriveConstants.DRIVE_SPEED, turnSpeed * DriveConstants.TURNING_SPEED_LIMIT);
        }
      else{
        if (inHighGear){
          if ((Math.abs(encoderL.getVelocity()) <= 1000) || (Math.abs(encoderR.getVelocity()) <= 1000))
            {
              differentialDrive.arcadeDrive(slewRateLimiter.calculate(forwardSpeed) * DriveConstants.DRIVE_SPEED, turnSpeed * -.75);
            }
          else{  
          differentialDrive.arcadeDrive(slewRateLimiter.calculate(forwardSpeed) * DriveConstants.DRIVE_SPEED_HIGH, turnSpeed * DriveConstants.TURNING_SPEED_HIGH);
          }
        }
      
       else{
        differentialDrive.arcadeDrive(slewRateLimiter.calculate(forwardSpeed) * DriveConstants.DRIVE_SPEED, turnSpeed * DriveConstants.TURNING_SPEED_LOW);
        


      }
    }
  }
  }

  private double driveTrainP() {
    error = encoderL.getPosition() - encoderR.getPosition();
    //integral += error*.02;
    return .1 * error;
  }

  public void driveStraight(double forwardSpeed) {
    differentialDrive.arcadeDrive(forwardSpeed, driveTrainP());
  }
  
  public void encoderReset() {
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  public double getEncoderPositionL(){
    return encoderL.getPosition();
  }
  public double getEncoderPositionR(){
    return encoderL.getPosition();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pigeon.getRotation2d(), 0,0,pose);
    //we know that the encoders' positions are both zero since we just set them in encoderReset();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts){
    SCG_L.setVoltage(leftVolts);
    SCG_R.setVoltage(rightVolts);
    differentialDrive.feed();
  }
  public CommandBase setVolts(double left, double right){
    return runOnce(() -> tankDriveVolts(left, right));
  }
  public CommandBase ShiftUp(){
    return runOnce(() -> upShift());
  }
  public CommandBase ShiftDown(){
    
    return runOnce(() -> downShift());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    //Driving speed in meters/second = 1/60 minutes per second * roatations-meters conversion constant * motor RPMs (encoder velocity)
    Double leftWheelMetersPerSecond = (1/60) * DriveConstants.ROTATIONS_TO_METERS * encoderL.getVelocity();
    Double rightWheelMetersPerSecond = (1/60) * DriveConstants.ROTATIONS_TO_METERS * encoderL.getVelocity();
    return new DifferentialDriveWheelSpeeds(leftWheelMetersPerSecond, rightWheelMetersPerSecond);
  }
  public boolean AutoBalance(){
    double roll_error = Math.toDegrees(pigeon.getRoll());//the angle of the robot
    double balance_kp = .001;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      differentialDrive.arcadeDrive(position_adjust, 0);//makes the robot move
      return false;
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      differentialDrive.arcadeDrive(position_adjust, 0);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      differentialDrive.arcadeDrive(0, 0);//stops the robot
      return true;}
    
  }

  public boolean AutoTurn(double setpoint){
    double yaw_error = pigeon.getYaw() - setpoint;
    double turning_kp = .01;
    double position_adjust = 0.0;
    double min_command = .01;
    if (yaw_error > 2.0)
    {
      position_adjust = turning_kp * yaw_error - min_command;
      differentialDrive.arcadeDrive(0, position_adjust);
      return false;
    }
    else if (yaw_error < 2.0)
    {
      position_adjust = turning_kp * yaw_error + min_command;
      differentialDrive.arcadeDrive(0, position_adjust);
      return false;
    }

    else{differentialDrive.arcadeDrive(0, 0);return true;}
    
  }
  public void setBrake() {
    //setting coast or brake mode, can also be done in Rev tuner
    motorFR.setIdleMode(IdleMode.kBrake);
    motorBR.setIdleMode(IdleMode.kBrake);
    motorFL.setIdleMode(IdleMode.kBrake);
    motorBL.setIdleMode(IdleMode.kBrake);
    BrakeMode = true;
  }
  public void setCoast() {
    //setting coast or brake mode, can also be done in Rev tuner
    motorFR.setIdleMode(IdleMode.kCoast);
    motorBR.setIdleMode(IdleMode.kCoast);
    motorFL.setIdleMode(IdleMode.kCoast);
    motorBL.setIdleMode(IdleMode.kCoast);
    BrakeMode = false;
  }
  public CommandBase setBrakeCommand() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    return runOnce(
      () -> {
        motorFR.setIdleMode(IdleMode.kBrake);
        motorBR.setIdleMode(IdleMode.kBrake);
        motorFL.setIdleMode(IdleMode.kBrake);
        motorBL.setIdleMode(IdleMode.kBrake);
        BrakeMode = true;

      });
  }

  public CommandBase setCoastCommand() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    return runOnce(
      () -> {
        motorFR.setIdleMode(IdleMode.kCoast);
        motorBR.setIdleMode(IdleMode.kCoast);
        motorFL.setIdleMode(IdleMode.kCoast);
        motorBL.setIdleMode(IdleMode.kCoast);
        BrakeMode = false;
      });
  }
  
  

  public CommandBase gyroReset() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    return runOnce(
      () -> {
          pigeon.reset();
      });
  }

  public CommandBase encoderResetCommand() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    return runOnce(
      () -> {
          encoderL.setPosition(0);
          encoderR.setPosition(0);
      });
  }
  @Override
  public void periodic() {
    //Automatic gear shifting
    //the speed of the left motor in RPMs
    double lvelocity = encoderL.getVelocity();
    //the speed of the right motor in RPMs
    double rvelocity = encoderR.getVelocity();

    SmartDashboard.putBoolean("HighGear?", inHighGear);
    SmartDashboard.putBoolean("Breaking?", BrakeMode);
    SmartDashboard.putNumber("LeftWheelSpeeds", encoderL.getVelocity());
    SmartDashboard.putNumber("RightWheelSpeeds", encoderR.getVelocity());
    SmartDashboard.putNumber("LeftEncoder", encoderL.getPosition());
    SmartDashboard.putNumber("RightEncoder", encoderR.getPosition());
    SmartDashboard.putNumber("Pigeon Roll", pigeon.getRoll());

   
    //* Automatic gear shifter - automatically shifts into high gear when the robot is driving fast enough and shifts into low gear when the robot slows down */
    //check if the robot is turning - if the speeds of the left and right motors are different
    boolean isTurning = Math.abs(lvelocity - rvelocity) < DriveConstants.TURN_THRESHOLD;
    //check if automatic shifitng is enabling and the robot IS NOT turning
/*     if(DriveConstants.AUTO_SHIFT_ENABLED){
      if(!isTurning){
        //if either motor exceeds the velocity threshold then shift into high gear
        if(Math.abs(lvelocity) > DriveConstants.UPSHIFT_THRESHOLD || Math.abs(rvelocity) > DriveConstants.UPSHIFT_THRESHOLD){
          upShift();
        }
        //if both motors' speeds are below the downshift threshold then shift down
        if(Math.abs(lvelocity) < DriveConstants.DOWNSHIFT_THRESHOLD && Math.abs(rvelocity) < DriveConstants.DOWNSHIFT_THRESHOLD){
          downShift();
        }
      } */
     
    
  }
  
}
