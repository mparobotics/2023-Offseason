// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardMeters extends CommandBase {
  
  private DriveSubsystem m_driveSubsystem;

  //the distance we want to drive in meters
  private double distanceToDrive;
  //the value the encoder will have when we reach the target.  
  private double target;
  
  //P (porportional) constant for driving to a target
  private double kP = 2;
  //maximum motor speed is 70%
  private double maxSpeed = 0.7;

  /** Creates a new DriveForward command.
   * @param setpoint - the distance to drive in meters 
   * @param ds - the robot's DriveSubsystem*/
  public DriveForwardMeters(double setpoint, DriveSubsystem ds) {
    //this command requires the DriveSubsystem
    addRequirements(ds);
    m_driveSubsystem = ds;

    //the distance we want to drive is selected with the "setpoint" input
    distanceToDrive = setpoint;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //target poisition = current position + desired distance
    target = m_driveSubsystem.getEncoderPosition() + distanceToDrive;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*the distance to the target. 
    Positive if target position is greater than current position, negative if target position is less than current position*/
    double distance = (target - m_driveSubsystem.getEncoderPosition());
    SmartDashboard.putNumber("target position", target);
    SmartDashboard.putNumber("distance to target", distance);
    
    //the speed is porportional to the distance from the target
    double speed = -kP * distance;

    //if the speed is greater than maxSpeed, then cap it at the maxSpeed
    speed = Math.min(maxSpeed, Math.max(-maxSpeed, speed));

    //if speed is really close to 0, set it to 0.3 if positive and -0.3 if negative. 
    //this ensures that the robot won't stop moving until it reaches the target
    if(Math.abs(speed) <  0.3){
      speed = Math.copySign(0.3, speed);
    }

    //drive forward at the calculated speed
    m_driveSubsystem.ArcadeDrive(speed,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the robot
    m_driveSubsystem.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If we are very close to the target, then end the command.
    return Math.abs(target - m_driveSubsystem.getEncoderPosition()) < 0.01;
    
  }
}
