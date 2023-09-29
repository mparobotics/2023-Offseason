
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
  private double m_speed;
  private final IntakeSubsystem m_IntakeSubsystem;
  /** Creates a new Intake. */
  public Intake(IntakeSubsystem intakeSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = intakeSubsystem;
    //m_speed = m_IntakeSubsystem.intakeTestSpeed;
    m_speed = speed;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.set(m_speed);
  }

  // Called once the command ends or is interrupted. 
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}