// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
/** Leo's swerve system */
public class SwerveDriveBase {
    
    //the swerve drivebase has four modules - each has a motor to drive forward
    //and a motor to spin the wheel to the correct direction
    private final SwerveModule wheel1;
    private final SwerveModule wheel2;
    private final SwerveModule wheel3;
    private final SwerveModule wheel4;
    

    /* the four modules are arranged like this:
   [ 3 ]-----------[ 2 ]
     |      Y        |
     |      |        |   
     |      0---> X  |
     |               |
     |               |
   [ 4 ]-----------[ 1 ]

     */
    
    public SwerveDriveBase(){
        //each module gets ids for a drive motor, turn motor, and an bsolute encoder as well as coefficients for a PID controller.
        
        wheel1 = new SwerveModule(DriveConstants.DRIVE_ID_1, DriveConstants.TURN_ID_1,DriveConstants.ENCODER_1);
        wheel2 = new SwerveModule(DriveConstants.DRIVE_ID_2, DriveConstants.TURN_ID_2,DriveConstants.ENCODER_2);
        wheel3 = new SwerveModule(DriveConstants.DRIVE_ID_3, DriveConstants.TURN_ID_3,DriveConstants.ENCODER_3);
        wheel4 = new SwerveModule(DriveConstants.DRIVE_ID_4, DriveConstants.TURN_ID_4,DriveConstants.ENCODER_4);
    }
    public SwerveSpeeds[] getStates(){
      SwerveSpeeds[] states = {wheel1.getSpeeds(),wheel2.getSpeeds(), wheel3.getSpeeds(), wheel4.getSpeeds()};

      return states;
    }
    public void setSpeeds(SwerveSpeeds speed1, SwerveSpeeds speed2, SwerveSpeeds speed3, SwerveSpeeds speed4, double orientation){
      wheel1.driveSpeed(speed1, orientation);
      wheel2.driveSpeed(speed2, orientation);
      wheel3.driveSpeed(speed3, orientation);
      wheel4.driveSpeed(speed4, orientation);
    }
    public void makeXposition(){
      wheel1.setAngle(315);
      wheel2.setAngle(45);
      wheel3.setAngle(135);
      wheel4.setAngle(225);
    }
    /**Drive the robot at a given x speed, y speed, and spin speed. 
     * swerve drive allows each of these parameters to be independent of each other.
     * orientation should be a heading from a gyroscope for field-aligned driving
      or 0 for robot-aligned driving*/
    public void SwerveDrive(double xSpeed, double ySpeed, double spinSpeed, double orientation){
      
    
      //use trigonometry to calculate the x-  and y- components of the spinning speeds.
      //This only needs to be calculated once, then we can reuse values in all four modules
      //sin(45deg) and cos(45deg) both equal sqrt(2)/2
      //then scale the amount by the turning speed
      double sinR45 = Math.sqrt(2)/2 * spinSpeed;
      //combine translation and rotation speed and store in swerveSpeeds
      SwerveSpeeds speed1 = SwerveSpeeds.XY(xSpeed + sinR45 , ySpeed + sinR45);
      SwerveSpeeds speed2 = SwerveSpeeds.XY(xSpeed - sinR45 , ySpeed + sinR45);
      SwerveSpeeds speed3 = SwerveSpeeds.XY(xSpeed - sinR45 , ySpeed - sinR45);
      SwerveSpeeds speed4 = SwerveSpeeds.XY(xSpeed + sinR45 , ySpeed - sinR45);
      //find the highest speed among all four wheels
      double maxSpeed = Math.max(Math.max(speed1.absSpeed, speed2.absSpeed),Math.max(speed3.absSpeed, speed4.absSpeed));
      if(maxSpeed > 1){
        //if any motors exceed 100% speed, scale all speeds by the same amount so that the fastest motor is at 100% 
        double scale = 1/maxSpeed;
          
        speed1.scaleBy(scale);
        speed2.scaleBy(scale);
        speed3.scaleBy(scale);
        speed4.scaleBy(scale);
      }
      //set drive speeds
      wheel1.driveSpeed(speed1, orientation);
      wheel2.driveSpeed(speed2, orientation);
      wheel3.driveSpeed(speed3, orientation);
      wheel4.driveSpeed(speed4, orientation);

      SmartDashboard.putNumber("Wheel 1 Direction", wheel1.getDirection());
      SmartDashboard.putNumber("Wheel 2 Direction", wheel2.getDirection());
      SmartDashboard.putNumber("Wheel 3 Direction", wheel3.getDirection());
      SmartDashboard.putNumber("Wheel 4 Direction", wheel4.getDirection());

      SmartDashboard.putNumber("Wheel 1 Speed", wheel1.getSpeed());
      SmartDashboard.putNumber("Wheel 2 Speed", wheel2.getSpeed());
      SmartDashboard.putNumber("Wheel 3 Speed", wheel3.getSpeed());
      SmartDashboard.putNumber("Wheel 4 Speed", wheel4.getSpeed());

    }

}
