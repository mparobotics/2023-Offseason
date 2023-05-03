// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.controller.PIDController;



import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.IntakeConstants;

import frc.robot.commands.ArcadeDrive;


import frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //the pneumatics to control the arm
  private final DoubleSolenoidSubsystem m_doublesolenoidSubsystem = new DoubleSolenoidSubsystem(); //replicating a double solenoid subsystem
  
  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  
  // The robot's subsystems and commands are defined here...
  //Creating instance of IntakeSubsystem called m_intakeSubsystem
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed

  //xbox controller
  private CommandJoystick flightStickL = new CommandJoystick(0);
  private CommandJoystick flightStickR = new CommandJoystick(1);

  public CommandJoystick box = new CommandJoystick(3);
  public Joystick safetyButton = new Joystick(2);

  //the drive subsystem
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();



  //individual pid controllers for the left and right sides of the robot
  PIDController leftController; 
  PIDController rightController; 

  

  
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    phCompressor.enableDigital();
    
    
    
    

    SmartDashboard.putString("THIS IS OUTREACH CODE!!!! ","DO NOT USE IN COMPETITIONS!!!");
    // Configure the trigger bindings
    configureBindings();


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

    
    //new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new ShiftUp(m_driveSubsystem));
    //new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ShiftDown(m_driveSubsystem));
    
    flightStickL.button(1).onTrue(m_driveSubsystem.ShiftDown());
  
    //xbox.button(Button.kLeftStick.value).whileTrue(new AutoTurn(m_driveSubsystem, 0));
    //xbox.button(Button.kRightStick.value).whileTrue(new AutoTurn(m_driveSubsystem, -180));
    flightStickL.button(2).whileTrue(m_driveSubsystem.setBrakeCommand()); // deprecated due to accidental presses
    flightStickR.button(2).whileTrue(m_driveSubsystem.setCoastCommand()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    //xbox.button(Button.kX.value).whileTrue(m_doublesolenoidSubsystem.shoot());
   // xbox.button(Button.kY.value).whileTrue(m_doublesolenoidSubsystem.retract());
    
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, 
    () -> flightStickL.getY(), () -> flightStickR.getX()));

    box.axisGreaterThan(1, .5).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED));
    //xbox.axisGreaterThan(Axis.kRightTrigger.value, 0.5).onFalse(m_driveSubsystem.setCoastCommand());
    //flightStickR.button(2).onTrue(m_driveSubsystem.setBrakeCommand());
    //flightStickR.button(2).onFalse(m_driveSubsystem.setCoastCommand());
    box.button(1).whileTrue(m_doublesolenoidSubsystem.retract()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(2).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED));
    box.button(3).whileTrue(m_doublesolenoidSubsystem.chuteintake());
    box.button(4).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.OUTTAKE_SPEED));
    box.button(5).whileTrue(m_doublesolenoidSubsystem.groundintake());
    box.button(6).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED));

    //helms.button(Button.kB.value).onTrue(m_driveSubsystem.gyroReset());
  }
  

  public Command SetCoast(){
    return m_driveSubsystem.setCoastCommand();
  }

  public Command SetBrake(){
    return m_driveSubsystem.setBrakeCommand();
  }


 public Command downShift(){
  return m_driveSubsystem.ShiftDown();
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    
    return null;
    


  }
  
};