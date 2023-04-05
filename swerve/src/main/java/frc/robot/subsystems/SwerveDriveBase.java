// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



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
        
        wheel1 = new SwerveModule(DriveConstants.DRIVE_ID_1, DriveConstants.TURN_ID_1,DriveConstants.ENCODER_1,1,-1);
        wheel2 = new SwerveModule(DriveConstants.DRIVE_ID_2, DriveConstants.TURN_ID_2,DriveConstants.ENCODER_2, 1, 1);
        wheel3 = new SwerveModule(DriveConstants.DRIVE_ID_3, DriveConstants.TURN_ID_3,DriveConstants.ENCODER_3,-1,1);
        wheel4 = new SwerveModule(DriveConstants.DRIVE_ID_4, DriveConstants.TURN_ID_4,DriveConstants.ENCODER_4,-1,-1);
    }
    public Coord2D[] getStates(){
      Coord2D[] states = {wheel1.getSpeeds(),wheel2.getSpeeds(), wheel3.getSpeeds(), wheel4.getSpeeds()};

      return states;
    }
    public void setSpeeds(Coord2D speed1, Coord2D speed2, Coord2D speed3, Coord2D speed4, double orientation){
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
      
      //get ideal speeds from each wheel
      Coord2D speed1 = wheel1.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
      Coord2D speed2 = wheel2.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
      Coord2D speed3 = wheel3.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
      Coord2D speed4 = wheel4.calculateIdealSpeeds(xSpeed, ySpeed, spinSpeed);
      

      
      //find the highest speed achieved by any of the four motors
      double maxSpeed = Math.max(Math.max(speed1.absLength(), speed2.absLength()),Math.max(speed3.absLength(), speed4.absLength()));

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

    }

}
