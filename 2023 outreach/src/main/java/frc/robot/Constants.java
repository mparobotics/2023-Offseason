// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public final class IntakeConstants 
  {
    //intake motor IDs
    public static final int INTAKE_MOTOR_R_ID = 14;
    public static final int INTAKE_MOTOR_L_ID = 15;
    //intake speeds
    public static final double INTAKE_SPEED = .3; //.35
    public static final double OUTTAKE_SPEED = -.3;
    public static final double SHOOTING_SPEED = -1;

  }

  public final class DriveConstants
  {
    public static final int MOTOR_FL_ID = 11;
    public static final int MOTOR_FR_ID = 13;
    public static final int MOTOR_BL_ID = 10;
    public static final int MOTOR_BR_ID = 12;

    public static final double LOW_TO_HIGH = 0.3332;
  
    public static final int PIGEON_ID = 17;


    //pneumatics compressor
    public static final int COMPRESSOR_ID = 0;

    public static final int SOLENOID_CHANNEL = 0;
  
    //gear shifter solenoids
    public static final int SHIFT_SOLENOID_CHANNEL = 9;
    public static final double MAX_DRIVE_SPEED = 4000;
   

    //driving speeds
    public static final double TURNING_SPEED_LIMIT = -.3;
    public static final double TURNING_SPEED_LOW = -.7;
    public static final double TURNING_SPEED_HIGH = 0;
    public static final double DRIVE_SPEED = -0.5;
    public static final double DRIVE_SPEED_HIGH = 0;

    //if the motor speed (RPMs) exceeds this value, then shift into high gear
    public static final double UPSHIFT_THRESHOLD = 5000;
    //if the motor speed (RPMs) gets below this value, then shift into low gear
    public static final double DOWNSHIFT_THRESHOLD = 1000;
    //difference between motor speeds needs to be below this threshold to be considered "not turning" (RPMs)
    public static final double TURN_THRESHOLD = 50;
    //if the robot should shift gears automatically
    public static final boolean AUTO_SHIFT_ENABLED = false;
    
    //how many encoder rotations = 1 meter of robot travel
    public static final double ROTATIONS_TO_METERS = 8 * Math.PI * 0.0254; //8in wheel diameter * PI * 0.0254 meters/inch

    //Autonomous trajectory following constants
    //! NOT TESTED or LAST YEAR'S values
    public static final double DRIVE_P_GAIN = 0;
    
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double DRIVE_KS = 0.6057;
    public static final double DRIVE_KV = 2.4263;
    public static final double DRIVE_KA = 0.37369;
    //the distance between the left and right sides of the robot (meters)
    public static final double TRACK_WIDTH_METERS = 0.73253;
    //Porportional gain value for driving straight
    public static final double DRIVE_STRAIGHT_P = 0.0007;

    public static final double ACCELERATION_RATE_LIMIT = 0;
    public static final double DECCELERATION_RATE_LIMIT = .5;
  
    } 
  public final class OperatorConstants
  {
    public static final int XBOX_CONTROLLER_ID = 0;
    public static final int BOX_ID = 1;
  }

  public final class AutoSelectorConstants
  {
    public static final String Test_Auto_1 = "0";
    public static final String Test_Auto_2 = "1";

    public static final String Pick_and_Score = "2";
    public static final String Score_Low_and_Leave = "3";
    public static final String Score_High_and_Leave = "4";
    public static final String Balance = "5";
    public static final String Leave = "6";
    public static final String Balance2Cube= "7";
     
  }
  public final static class LEDConstants{
    public static final int CANDLE_ID = 18;
    public static final int LED_COUNT = 8;

    public static final int[] PURPLE_RGB = {188,0,255};
    public static final int[] YELLOW_RGB = {255,255,0};
  } 
}
