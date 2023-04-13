// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kTrajectory;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveDriveBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveDriveBase m_SwerveDrive = new SwerveDriveBase();

  private XboxController xbox = new XboxController(0);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_SwerveDrive.setDefaultCommand(new SwerveDrive(() -> xbox.getLeftY(), () -> -xbox.getLeftX(), () -> xbox.getRightX(), m_SwerveDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  
  SequentialCommandGroup followTrajectory(String file){
    return m_SwerveDrive.followTrajectory(file);
  }
  SequentialCommandGroup followTrajectory(Trajectory t){
    return m_SwerveDrive.followTrajectory(t);
  }
  public SequentialCommandGroup makeTrajectory(Pose2d start, Pose2d end){
    TrajectoryConfig config = new TrajectoryConfig(kTrajectory.MAX_VELOCITY, kTrajectory.MAX_ACCELERATION);
    return followTrajectory(TrajectoryGenerator.generateTrajectory(start,List.of(),end,config));
  }

  
  SequentialCommandGroup SimpleTestAuto(){
    return new SequentialCommandGroup(
      makeTrajectory(
        new Pose2d(), 
        new Pose2d(3,0,Rotation2d.fromDegrees(0))
      )
    );
    
  }


  
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return SimpleTestAuto();
  }
}
