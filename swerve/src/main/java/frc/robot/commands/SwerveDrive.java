// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends CommandBase {
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier r;

  private DriveSubsystem m_DriveSubsystem;
  /** Creates a new SwerveDrive. */
  public SwerveDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier spinSpeed, DriveSubsystem drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.x = xSpeed;
    this.y = ySpeed;
    this.r = spinSpeed;

    m_DriveSubsystem = drivesub;
    addRequirements(drivesub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.setDriveSpeedSwerve(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());
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
