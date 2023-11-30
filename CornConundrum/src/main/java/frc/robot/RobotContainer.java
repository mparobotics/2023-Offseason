// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveForwardMeters;
import frc.robot.commands.TurnAround;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //private final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  enum AutoChoices{
    NOTHING, LEAVE_BARN, SPOILED_COBB_L, SPOILED_COBB_R
  }
  
  private final SendableChooser<AutoChoices> auto_selector = new SendableChooser<AutoChoices>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    auto_selector.setDefaultOption("Do Nothing", AutoChoices.NOTHING);
    auto_selector.addOption("Leave Barn", AutoChoices.LEAVE_BARN);
    auto_selector.addOption("Get Spoiled Cobb L", AutoChoices.SPOILED_COBB_L);
    auto_selector.addOption("Get Spoiled Cobb R", AutoChoices.SPOILED_COBB_R);
    SmartDashboard.putData("Auto Chooser", auto_selector);
    SmartDashboard.putString("Leave Barn Auto", "goes forward 1 meter");
    SmartDashboard.putString("Get Spoiled Cob L Auto", "leaves barn, turns around left, goes back");
    SmartDashboard.putString("Get Spoiled Cob R Auto", "leaves barn, turns around right, goes back");
    SmartDashboard.putString("Do nothing auto", "what do you think this does?");


    // Configure the trigger bindings
    configureBindings();
  }
  public void periodic(){
   
    
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
  //decide what events trigger what commands
  private void configureBindings() {
    //Set DriveSubsystem to be always driving by default
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> 
    //Drive the robot. Left joystick up/down position controls speed, right joystick left/right postition controls turning
    m_driveSubsystem.ArcadeDrive(xbox.getLeftY(),xbox.getRightX()), m_driveSubsystem
    ));
   
    
    //Left trigger dumps cargo
    //new Trigger(() -> (xbox.getRightTriggerAxis() > 0.5)).onTrue(m_pneumatics.in());
    //new Trigger(() -> (xbox.getRightTriggerAxis() > 0.5)).onFalse(m_pneumatics.out());

    //A,B,X,and Y buttons set the LED colors to green, red, blue, and yellow.
    xbox.button(Button.kA.value).onTrue(m_driveSubsystem.setColor(0,255,0));
    xbox.button(Button.kB.value).onTrue(m_driveSubsystem.setColor(255,0,0));
    xbox.button(Button.kX.value).onTrue(m_driveSubsystem.setColor(0,0,255));
    xbox.button(Button.kY.value).onTrue(m_driveSubsystem.setColor(255,180,0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoChoices auto = auto_selector.getSelected();
    switch(auto){
      case NOTHING:
        return null;
      case LEAVE_BARN:
        return new DriveForwardMeters(1, m_driveSubsystem);
      case SPOILED_COBB_L:
        //Leave the barn and push a spoiled cobb back into the barn
        return new SequentialCommandGroup(
        //drive forward
        new DriveForwardMeters(2, m_driveSubsystem),
        //turn around in an arc
        new TurnAround(m_driveSubsystem, -180),
        //drive back
        new DriveForwardMeters(2, m_driveSubsystem));
      case SPOILED_COBB_R:
        //Leave the barn and push a spoiled cobb back into the barn
        return new SequentialCommandGroup(
        //drive forward
        new DriveForwardMeters(2, m_driveSubsystem),
        //turn around in an arc
        new TurnAround(m_driveSubsystem, 180),
        //drive back
        new DriveForwardMeters(2, m_driveSubsystem));
    
    }
    return null;

    


  }
}
