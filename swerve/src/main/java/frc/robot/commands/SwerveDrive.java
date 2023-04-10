// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDriveBase;

public class SwerveDrive extends CommandBase {
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier r;

  private SwerveDriveBase m_SwerveDrive;
  /** Creates a new SwerveDrive. */
  public SwerveDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier spinSpeed, SwerveDriveBase drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.x = xSpeed;
    this.y = ySpeed;
    this.r = spinSpeed;

    m_SwerveDrive = drivesub;
    addRequirements(drivesub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = x.getAsDouble();
    double ySpeed = y.getAsDouble();
    double spinSpeed = r.getAsDouble();

    //deadbanding
    if(Math.abs(xSpeed) < 0.1){ xSpeed = 0; }
    if(Math.abs(ySpeed) < 0.1){ ySpeed = 0; }
    if(Math.abs(spinSpeed) < 0.1){ ySpeed = 0; }
    
    xSpeed *= DriveConstants.DRIVE_SPEED;
    ySpeed *= DriveConstants.DRIVE_SPEED;

    spinSpeed *= DriveConstants.TURN_SPEED;
   

  
    m_SwerveDrive.SwerveDrive(xSpeed, ySpeed, spinSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
