// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DoubleSolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDriveBalance;
import frc.robot.commands.AutoDriveBangBang;
import frc.robot.commands.AutoDriveBangBangLow;
import frc.robot.commands.AutoDriveBangBangStraight;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoIntakeInstant;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DownShift;
import frc.robot.commands.Intake;
import frc.robot.commands.NullCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsubsystem;

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
  private final LEDsubsystem m_ledSubsystem = new LEDsubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  //xbox controller
  private CommandJoystick flightStickL = new CommandJoystick(0);
  private CommandJoystick flightStickR = new CommandJoystick(1);

  private CommandJoystick box = new CommandJoystick(2);

  //the drive subsystem
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  //moving the drive kinematics from Constants to DriveSubsystem fixed the static issue
  private  DifferentialDriveKinematics DRIVE_KINEMATICS =
       new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

  //individual pid controllers for the left and right sides of the robot
  PIDController leftController; 
  PIDController rightController; 

  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  //these auto selector strings can be anything as long as they are unique
  private final String Pick_and_Score = "1";
  private final String Balance2Cube= "2";
  private final String TwoPiecesNoBalance = "3";
  private final String DoNothing = "4";
  private final String JustShoot = "5";
  private final String TwoPiecesHighNoBalance = "6";
  private final String Balance1Cube= "7";
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    phCompressor.enableDigital();
    
    
    m_chooser.addOption("Score 2 Pieces", TwoPiecesNoBalance);
    m_chooser.addOption("Score 1 High, and another, no balance", TwoPiecesHighNoBalance);
    m_chooser.addOption("Score 2 Cubes & Balance", Balance2Cube);
    m_chooser.addOption("Score 1 Cube High & Balance", Balance1Cube);
    m_chooser.addOption("Do Nothing", DoNothing);
    m_chooser.setDefaultOption("Just Shoot", JustShoot);
    

    SmartDashboard.putData("Auto Chooser", m_chooser);
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
    //A button shifts the gearbox into high gear
    flightStickL.button(1).onTrue(m_driveSubsystem.ShiftDown());
    //B button shifts the gearbox into low gear
    flightStickR.button(1).onTrue(m_driveSubsystem.ShiftUp());
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
    box.button(3).whileTrue(m_doublesolenoidSubsystem.retract()); // when b is pressed, it calls the forwardSolenoid command that is inside the double solenoid subsystem which makes it go forward.
    box.button(10).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED));
    box.button(2).whileTrue(m_doublesolenoidSubsystem.chuteintake());
    box.button(8).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.OUTTAKE_SPEED));
    box.button(1).whileTrue(m_doublesolenoidSubsystem.groundintake());
    box.button(7).whileTrue(new Intake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED));

    //helms.button(Button.kB.value).onTrue(m_driveSubsystem.gyroReset());
  }

  /**
   takes a location of the JSON file as an input
   and generates a ramsete command from the file
   */
  private RamseteCommand followTrajectory(String filePath){
    //trajectory object
    Trajectory pTrajectory = new Trajectory();
    //try to open the file
    try{
      Path TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
      pTrajectory = TrajectoryUtil.fromPathweaverJson(TrajectoryPath);
    }
    catch(IOException ex){
      DriverStation.reportError("Unable to open trajectory:" + filePath, ex.getStackTrace());
    }
    
    //creates the ramsete command
    RamseteCommand rCommand = new RamseteCommand(
        pTrajectory,
        m_driveSubsystem::getPose,
        new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
        DRIVE_KINEMATICS,
        m_driveSubsystem::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        m_driveSubsystem::tankDriveVolts,
        m_driveSubsystem
      );
    
    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(pTrajectory.getInitialPose());
   
    return rCommand;

  }
  
  
 
  private Command runIntaking(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.INTAKE_SPEED).withTimeout(seconds);
  }
  private Command runOuttaking(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.OUTTAKE_SPEED).withTimeout(seconds);
  }
  private Command runShooting(double seconds){
    return new AutoIntake(m_intakeSubsystem, IntakeConstants.SHOOTING_SPEED).withTimeout(seconds);
  }

  private Command runShootingSlow(double seconds){
    return new AutoIntake(m_intakeSubsystem, -.8).withTimeout(seconds);
  }
  private Command AutoDrive(double setpoint, double speed){
    return new AutoDriveBangBang(m_driveSubsystem, setpoint, speed);
  }

  private Command AutoDriveStraight(double setpoint, double speed){
    return new AutoDriveBangBangStraight(m_driveSubsystem, setpoint, speed);
  }

  private Command AutoDriveLow(double setpoint, double speed){
    return new AutoDriveBangBangLow(m_driveSubsystem, setpoint, speed);
  }

  private ParallelCommandGroup AutoDriveWithIntakeDrop(double setpoint, double speed){
    return new ParallelCommandGroup(AutoDrive(setpoint, speed), setArmGroundWithDelay()) ;
  }

  private Command AutoDrive1(double setpoint, double speed){
    return new AutoDriveBangBang(m_driveSubsystem, setpoint, speed);
  }

  private Command downShift(){
    return new DownShift(m_driveSubsystem);
  }

  public Command SetCoast(){
    return m_driveSubsystem.setCoastCommand();
  }

  public Command SetBrake(){
    return m_driveSubsystem.setBrakeCommand();
  }


  private Command autoDriveBalance(){
    return new AutoDriveBalance(m_driveSubsystem);
  }

  public Command setLedCube(){
    return m_ledSubsystem.Cube();
  }

  private Command setLedCone(){
    return m_ledSubsystem.Cone();
  }

  private Command setArmGround(){
    return m_doublesolenoidSubsystem.groundintake();
  }

  private Command nullCommand(){
    return new NullCommand();
  }

  private SequentialCommandGroup setArmGroundWithDelay(){
    return new SequentialCommandGroup(nullCommand().withTimeout(3.5), setArmGround());
  }

  private Command encoderReset(){
    return m_driveSubsystem.encoderResetCommand();
  }

  private Command setArmRetracted(){
    return m_doublesolenoidSubsystem.retract();
  }
  private Command setArmShoot(){
    return m_doublesolenoidSubsystem.shoot();
  }

private Command autoIntakeInstant(double speed){
  return new AutoIntakeInstant(m_intakeSubsystem, speed);
}

  private Command stopRobot(){
    return m_driveSubsystem.setVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    
    //troubleshooting info
    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left reference");
    var leftMeasurement = table.getEntry("left measurement");
    var rightReference = table.getEntry("right reference");
    var rightMeasurement = table.getEntry("right measurement");
    
    //Set PID controllers
    leftController = new PIDController(DriveConstants.DRIVE_P_GAIN, 0, 0);
    rightController = new PIDController(DriveConstants.DRIVE_P_GAIN, 0, 0);
    
    //the loaction of a JSON trajectory file
/*     String Trajectory_pickandscore1 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (1).wpilib.json";
    String Trajectory_pickandscore2 = "pathplanner/generatedJSON/1,2,3 - Pick Up & Score (2).wpilib.json";
    String Trajectory_leave = "pathplanner/generatedJSON/1,2,3 - Leave.wpilib.json";
    String Test_Auto = "pathplanner/generatedJSON/Test Auto.json"; */
    
    //display values in the table
    leftMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
    leftReference.setNumber(leftController.getSetpoint());
    rightMeasurement.setNumber(m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
    rightReference.setNumber(rightController.getSetpoint());
    
    
     m_autoSelected = m_chooser.getSelected();
     System.out.println("Auto Selected: " + m_autoSelected);
    
    //if (m_autoSelected != null){
      switch (m_autoSelected)
      {
        //why do we always encoder reset before driving?
        //can we make the encoderReset() a part of AutoDrive1() or will that break something?
          case TwoPiecesNoBalance:
          //set against grid
            return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftUp(), m_driveSubsystem.setBrakeCommand(), runShooting(.5),autoIntakeInstant(IntakeConstants.INTAKE_SPEED),
            setArmGround(), encoderReset(),
            AutoDriveStraight(220 * DriveConstants.LOW_TO_HIGH, .4), setArmRetracted(), encoderReset(), AutoDriveStraight(-200 * DriveConstants.LOW_TO_HIGH, -.4),
            runShooting(.7), encoderReset(), setArmGround(), AutoDriveStraight(180 * DriveConstants.LOW_TO_HIGH, .5));

            case TwoPiecesHighNoBalance:
            //set against grid
            return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftUp(), m_driveSubsystem.setBrakeCommand(),
            runIntaking(.5), runShooting(.5), autoIntakeInstant(IntakeConstants.INTAKE_SPEED),
            setArmGround(), nullCommand().withTimeout(.3), encoderReset(),
            AutoDriveStraight(220 * DriveConstants.LOW_TO_HIGH, .5), setArmRetracted(), encoderReset(), AutoDriveStraight(-200 * DriveConstants.LOW_TO_HIGH, -.4),
            runShooting(.7), encoderReset(), setArmGround(), AutoDriveStraight(180 * DriveConstants.LOW_TO_HIGH, .4));

          case Balance2Cube:
          //set against charging station
            return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftUp(),m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.setBrakeCommand(), runShooting(1), encoderReset(),
            autoIntakeInstant(IntakeConstants.INTAKE_SPEED), AutoDriveWithIntakeDrop(200 * DriveConstants.LOW_TO_HIGH, .5),  
            autoIntakeInstant(0), setArmRetracted(), encoderReset(), m_driveSubsystem.setBrakeCommand(), downShift().withTimeout(2), AutoDriveLow(-140, -.5),
            autoIntakeInstant(IntakeConstants.SHOOTING_SPEED), (autoDriveBalance()), new NullCommand().withTimeout(1),
            autoIntakeInstant(0));

            case Balance1Cube:
            //set against charging station
              return new SequentialCommandGroup(m_driveSubsystem.setBrakeCommand(), m_driveSubsystem.ShiftUp(), m_driveSubsystem.setBrakeCommand(),
              runIntaking(.7), runShooting(.6), autoIntakeInstant(0), encoderReset(),
              AutoDriveStraight(200 * DriveConstants.LOW_TO_HIGH, .5), downShift(), new NullCommand().withTimeout(2),  
              encoderReset(), m_driveSubsystem.setBrakeCommand(), AutoDriveLow(-105, -.5),
              (autoDriveBalance()), new NullCommand().withTimeout(1),
              autoIntakeInstant(0));
      
        case DoNothing:
          return null;

        case JustShoot:
          return runShooting(2);
        }
    //} 
     
    //else {return null;}
    return null;
    


  }
  
};