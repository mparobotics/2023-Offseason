// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.io.IOException;
import java.nio.file.Path;


import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kTrajectory;


/** Leo's swerve system */
public class SwerveDriveBase extends SubsystemBase{
    
    
  //the swerve drivebase has four modules - each has a motor to drive forward
  //and a motor to spin the wheel to the correct direction
  private final SwerveModule wheel1;
  private final SwerveModule wheel2;
  private final SwerveModule wheel3;
  private final SwerveModule wheel4;

  //a Pigeon2 gyro measures the direction of the robot
  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(kDrive.GYRO_ID);
    
    
  
  //positions of the wheels relative to the center of the robot (or the center of rotation of the robot, if you want it to rotate off center for some reason)
  private Translation2d wheel1pos = new Translation2d(1,1);
  private Translation2d wheel2pos = new Translation2d(-1,1);
  private Translation2d wheel3pos = new Translation2d(-1,-1);
  private Translation2d wheel4pos = new Translation2d(1,-1);
  //the distance from the farthest module to the center
  private double maxLength;

  //swerve kinematics contains the layout of the four modules
  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(wheel1pos, wheel2pos, wheel3pos, wheel4pos);
  //odometry calculates where the robot is on the field based on data from the four modules and the gyro
  private SwerveDriveOdometry odometry;

  //if the robot should use field-aligned or robot-aligned coordinates
  public Boolean isFieldAligned = true;

  //the Pose2d that the robot is at when it's enabled
  //starting at (0,0) with a heading of 0 degrees
  private Pose2d startingPos = new Pose2d(0,0,new Rotation2d(Math.toRadians(0)));

  public SwerveDriveBase(){
      //each module gets ids for a drive motor, turn motor, and an absolute encoder as well as a relative position to the center of the robot.
      wheel1 = new SwerveModule(kDrive.DRIVE_ID_1, kDrive.TURN_ID_1,kDrive.ENCODER_1,wheel1pos);
      wheel2 = new SwerveModule(kDrive.DRIVE_ID_2, kDrive.TURN_ID_2,kDrive.ENCODER_2, wheel2pos);
      wheel3 = new SwerveModule(kDrive.DRIVE_ID_3, kDrive.TURN_ID_3,kDrive.ENCODER_3, wheel3pos);
      wheel4 = new SwerveModule(kDrive.DRIVE_ID_4, kDrive.TURN_ID_4,kDrive.ENCODER_4,wheel4pos);

      //scale the wheel speeds to be porportional to their distances from the center (in most cases the distances are all the same, so this doesn't affect anything)
      maxLength = Math.max(Math.max(wheel1pos.getNorm(),wheel2pos.getNorm()),Math.max(wheel3pos.getNorm(),wheel4pos.getNorm()));

      //initialize the odometry with the kinematics, the gyro, an array of module distances and angles, and the starting pose
      odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(), getPositions(),startingPos);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }


  /*gets all four module positions in an array */
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = {wheel1.getPosition(),wheel2.getPosition(), wheel3.getPosition(), wheel4.getPosition()};

    return positions;
  }
  /*sets the states (target angle and speed) of all four modules
   * this method is mainly meant to be used by the trajectory commands to drive the modules along the trajectory
  */
  public void setStates(SwerveModuleState[] states){
    wheel1.driveSpeedMeters(states[0], getDirection());
    wheel2.driveSpeedMeters(states[1], getDirection());
    wheel3.driveSpeedMeters(states[2], getDirection());
    wheel4.driveSpeedMeters(states[3], getDirection());

  }

  /*moves the wheels into an X shape to prevent sliding when balancing on the charging station*/
  public void makeXposition(){
    wheel1.setAngle(wheel1pos.getAngle().getDegrees());
    wheel2.setAngle(wheel2pos.getAngle().getDegrees());
    wheel3.setAngle(wheel3pos.getAngle().getDegrees());
    wheel4.setAngle(wheel4pos.getAngle().getDegrees());
  }
  /*updates the position estimate with new data */
  public void updateOdometry(){
    odometry.update(pigeon.getRotation2d(), getPositions());
  }
  /*gets the current position estimate of the robot */
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  /*gets the current direction of the robot in degrees - assuming it was at 0deg when it was enabled*/
  public double getDirection(){
    return pigeon.getYaw();
  }
  //if we want to use robot aligned coordinates, then pretend like the robot is always at 0deg
  private double alignedHeading(){
    return (isFieldAligned)? pigeon.getYaw(): 0;
  }

  /**Drive the robot at a given x speed, y speed, and spin speed. 
   * swerve drive allows each of these parameters to be independent of each other.
    or 0 for robot-aligned driving*/
  public void SwerveDrive(double xSpeed, double ySpeed, double spinSpeed){

    //get ideal speeds from each wheel - figure out how fast the motors should be spinning in what direction
    //calculated as a 2d vector with an x and y component. the length of the vector is the speed of the motor
    Translation2d speed1 = wheel1.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
    Translation2d speed2 = wheel2.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
    Translation2d speed3 = wheel3.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
    Translation2d speed4 = wheel4.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
      
    //sometimes the ideal speed of a wheel is faster than the maximum speed of the wheel (speed > 1),
    //if that is the case we need to scale all four speeds by the same amount so that only the fastest motors are spinning at 100%

    //find the highest speed achieved by any of the four motors
    double maxSpeed = Math.max(Math.max(speed1.getNorm(), speed2.getNorm()),Math.max(speed3.getNorm(), speed4.getNorm()));

      
    if(maxSpeed > 1){
      //if any motors exceed 100% speed, scale all speeds by the same amount so that the fastest motor is at 100% 
      double scale = 1/maxSpeed;
          
      speed1.times(scale);
      speed2.times(scale);
      speed3.times(scale);
      speed4.times(scale);
    }
    speed1.times(wheel1.ScaleSpeed(maxLength));
    speed2.times(wheel2.ScaleSpeed(maxSpeed));
    speed3.times(wheel3.ScaleSpeed(maxSpeed));
    speed4.times(wheel4.ScaleSpeed(maxSpeed));
    //set drive speeds
    //using the gyro heading we can do field-aligned driving instead of robot-aligned
    wheel1.driveSpeed(speed1, alignedHeading());
    wheel2.driveSpeed(speed2, alignedHeading());
    wheel3.driveSpeed(speed3, alignedHeading());
    wheel4.driveSpeed(speed4, alignedHeading());

  }
  
  public SequentialCommandGroup followTrajectory(Trajectory traj){

    PIDController Xpid = new PIDController(kTrajectory.kP_X, 0, 0);
    PIDController Ypid = new PIDController(kTrajectory.kP_Y, 0, 0);
    ProfiledPIDController Dpid = new ProfiledPIDController(kTrajectory.kP_D, 0, 0,null);
    Dpid.enableContinuousInput(0, 2 * Math.PI);
      
    //creates the swerve controller command using the trajectory
    SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
        traj,
        this::getPose, 
        kinematics, 
        Xpid,
        Ypid,
        Dpid, 
        this::setStates);
    SequentialCommandGroup cmd = new SequentialCommandGroup(swerveCommand);
    return cmd; 
  }


  public SequentialCommandGroup followTrajectory(String filepath){
    //trajectory object
    Trajectory pTrajectory = new Trajectory();
    //try to open the file
    try{
        Path TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        pTrajectory = TrajectoryUtil.fromPathweaverJson(TrajectoryPath);
    }
    catch(IOException ex){
        DriverStation.reportError("Unable to open trajectory:" + filepath, ex.getStackTrace());
    }

    return followTrajectory(pTrajectory);
  }
}
