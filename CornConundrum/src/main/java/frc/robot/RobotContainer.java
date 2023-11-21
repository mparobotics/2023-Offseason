// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  

  //a DriveSubsystem controls the 4 driving motors
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();



  //An xbox controller is used to control the robot
  private final CommandXboxController xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  //decide what events trigger what commands
  private void configureBindings() {
    //Set DriveSubsystem to be always driving by default
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> 
    //Drive the robot. Left joystick up/down position controls speed, right joystick left/right postition controls turning
    m_driveSubsystem.ArcadeDrive(xbox.getLeftY(),xbox.getRightX()), m_driveSubsystem
    ));
   
    xbox.button(Button.kA.value).onTrue(m_driveSubsystem.setColor(0,255,0));
    xbox.button(Button.kB.value).onTrue(m_driveSubsystem.setColor(255,0,0));
    xbox.button(Button.kX.value).onTrue(m_driveSubsystem.setColor(0,0,255));
    //xbox.button(Button.kY.value).onTrue(m_driveSubsystem.setColor(255,180,0));

    xbox.button(Button.kY.value).onTrue(m_pneumaticsSubsystem.toggle()); 

  }

  private Command encoderReset(){
    return m_driveSubsystem.encoderResetCommand();
  }
  
  public Command getAutonomousCommand() {
    //this robot does nothing in autonomous
    return null;
  }
}
