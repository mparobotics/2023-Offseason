// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class kDrive{
    //teleop driving speeds
    public static final double DRIVE_SPEED = 1;
    public static final double TURN_SPEED = 1;

    //current limits
    public static final int DRIVE_CURRENT_LIMIT = 80;
    public static final int TURN_CURRENT_LIMIT = 20;
    
    public static final double VOLTAGE_COMPENSATION = 12;



    //gear ratios and conversions
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //4 inch wheels - but all units in meters for consistency
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //circumference = PI * diameter

    public static final double HALF_WIDTH = Units.inchesToMeters(17.5) / 2; //distance from center to the side of the robot
    public static final double HALF_LENGTH = Units.inchesToMeters(20.5) / 2; //distance from center to the front of the robot

    public static final double DRIVE_GEAR_RATIO = 14/50 * 25/19 * 15/45; // ~8.14 motor rotation = 1 wheel rotation
    public static final double TURN_GEAR_RATIO = 12.8/1; //12.8 motor rotations = 1 full wheel swivel

    //conversion ratios for each type of encoder
    public static final double ENCODER_TICKS_TO_DEGREES = 360/TURN_GEAR_RATIO; //neo encoders 1 tick per rotation
    public static final double ABSOLUTE_TICKS_TO_DEGREES = 1; //absolute encoders are configured so 1 tick = 1 degree
    
    public static final double ENCODER_TICKS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO; //theoretical encoder ticks to meters - assumes no slipping

    //the fastest the robot can go
    public static final double MAX_SPEED = 4.5; // meters/second
    //PID values for the turn motors
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kIz = 0;
    public static final double kFF = 0;

    //PID values for the drive motors
    public static final double D_kP = 0.1;
    public static final double D_kI = 0;
    public static final double D_kD = 0;

    public static final double D_kIz = 0;
    public static final double D_kFF = 0;

    //feedforward constants - calculated in sysid characterization
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    //motors go in counterclockwise order starting with the front right motor as #1

  
    //IDs for the components of the swerve drive

    //this pigeon2 gyroscope measures the orientation of the robot
    public static final int GYRO_ID = 17;
  

    //these motors power the wheels
    public static final int DRIVE_ID_1 = 1;
    public static final int DRIVE_ID_2 = 3;
    public static final int DRIVE_ID_3 = 5;
    public static final int DRIVE_ID_4 = 7;

    //these motors spin the wheels to the correct angle
    public static final int TURN_ID_1 = 2;
    public static final int TURN_ID_2 = 4;
    public static final int TURN_ID_3 = 6;
    public static final int TURN_ID_4 = 8;

    //these absolute encoders remember the position of the wheels even when the robot is turned off
    public static final int ENCODER_1 = 9;
    public static final int ENCODER_2 = 10;
    public static final int ENCODER_3 = 11;
    public static final int ENCODER_4 = 12;


  }
  public static class kTrajectory{
    //trajectory tracking uses a P contoller for each axis of motion X, Y, rotation - each has its own kp
    public static final double kP_X = 0; 
    public static final double kP_Y = 0;
    public static final double kP_D = 0;

    

    public static final double MAX_VELOCITY = 3; // meters/seconds
    public static final double MAX_ACCELERATION = 1; // meters/seconds^2

  }
  public static class kOperator {
    public static final int kDriverControllerPort = 0;
  }
}
