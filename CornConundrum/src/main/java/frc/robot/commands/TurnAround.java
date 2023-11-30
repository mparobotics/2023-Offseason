// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class TurnAround  extends CommandBase {
  /** Creates a new TurnAround . */
  private DriveSubsystem m_driveSubsystem;
  private double target;

  private double kP = 2;
  private double maxSpeed = 0.5;

  public TurnAround (DriveSubsystem ds) {
    addRequirements(ds);
    m_driveSubsystem = ds;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = m_driveSubsystem.getAngle() + 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = (target - m_driveSubsystem.getAngle());
    
    
    double speed = -kP * distance;
    speed = Math.min(maxSpeed, Math.max(-maxSpeed, speed));
    if(Math.abs(speed) <  0.3){
      speed = Math.copySign(0.3, speed);
    }
    m_driveSubsystem.TankDrive(speed,0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target - m_driveSubsystem.getEncoderPosition()) < 0.01;
    
  }
}
