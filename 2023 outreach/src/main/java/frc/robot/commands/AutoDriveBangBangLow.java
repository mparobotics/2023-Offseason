// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveBangBangLow extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private double m_setpoint;
  private double startEncoderL;
  private double startEncoderR;
  private double m_speed;
  private double distanceFromSetpoint;
  private double AutoDriveKp = .032;
  /** Creates a new AutoDriveBangBang. */
  public AutoDriveBangBangLow(DriveSubsystem driveSub, double setpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSub;
    m_setpoint = setpoint;
    
    m_speed = speed;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.downShift();
    m_driveSubsystem.encoderReset();
    startEncoderL = m_driveSubsystem.getEncoderPositionL();
    startEncoderR = m_driveSubsystem.getEncoderPositionR();
    distanceFromSetpoint = (Math.abs(m_setpoint) - Math.abs(m_driveSubsystem.getEncoderPositionL() - startEncoderL));
    SmartDashboard.putNumber("AutoSetpoint", m_setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //we want to find how much more displacement is needed to get to our total amount of displacement when making this formula
    //displacement is the absolute value of a distance traveled.
    //if i travel either 1 or -1 units, the displacement is still 1
    //we can further define displacement as the distance from the starting point
    //for example, if I start at 3 and end at -2, my displacement is 5. How would we calculate that?
    //We can take where we end up (-2), and subtract 3, getting -5, which is vector of our travel (vectors have magnitude and direction)
    //but we only want the magnitude of that because we want to know how much further we have left to travel, 
    //so we use absolute value
      
    //we also only want the absolute value of the setpoint as that would be the total distance needed to be travled.

    //by subtracting how much we want to be displaced with how much we have been displaced already, we are left with how much
    //we still need to be displaced. For example, if I start at 3 and want to move  -5 units (to -2), and am at 1,
    //we would get distanceFromSetpoint = |-5| - |1 - 3| = 3. 
    distanceFromSetpoint = (Math.abs(m_setpoint) - Math.abs(m_driveSubsystem.getEncoderPositionL() - startEncoderL));
    SmartDashboard.putNumber("DistanceFromSetpoint", distanceFromSetpoint);
    if (distanceFromSetpoint < 10) {
      //we still have to give our value a magnitude again to travel in the correct direction, thus the inversion here.
      if (m_setpoint > 0) {m_speed = AutoDriveKp * distanceFromSetpoint + .175;}
      else {m_speed = -AutoDriveKp * distanceFromSetpoint - .175;}
      
    }

    m_driveSubsystem.setDriveSpeedArcade(-m_speed, 0);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDriveSpeedArcade(0, 0);
  }                           

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceFromSetpoint < 3;
  }
}
