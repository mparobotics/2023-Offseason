// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPIDSubsystem;

public class RunPID extends CommandBase {
  /** Creates a new RunPID. */
  private final NeoPIDSubsystem m_pidSubsystem;
  private DoubleSupplier m_setpoint;

  public RunPID(DoubleSupplier setpoint, NeoPIDSubsystem npid) {
    m_setpoint = setpoint;
    m_pidSubsystem = npid;
    addRequirements(npid);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pidSubsystem.setNEOSetpoint(m_setpoint.getAsDouble());

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
