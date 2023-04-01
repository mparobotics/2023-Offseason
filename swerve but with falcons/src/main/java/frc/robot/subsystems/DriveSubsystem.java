// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private SwerveDriveBase m_swerveDrive = new SwerveDriveBase();
 
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);

   /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

  }
  public void setDriveSpeedSwerve(double xSpeed, double ySpeed, double spinSpeed){
    //deadbanding
    if(Math.abs(xSpeed) < 0.1){ xSpeed = 0; }
    if(Math.abs(ySpeed) < 0.1){ ySpeed = 0; }
    if(Math.abs(spinSpeed) < 0.1){ ySpeed = 0; }
    
    xSpeed *= DriveConstants.DRIVE_SPEED;
    ySpeed *= DriveConstants.DRIVE_SPEED;

    spinSpeed *= DriveConstants.TURN_SPEED;

    //set the drive speed
    m_swerveDrive.SwerveDrive(xSpeed, ySpeed, spinSpeed, pigeon.getYaw());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
