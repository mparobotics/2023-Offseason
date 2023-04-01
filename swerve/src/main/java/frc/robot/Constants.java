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
public final class Constants {
  public static class DriveConstants{
    public static final double DRIVE_SPEED = 1;
    public static final double TURN_SPEED = 1;

    //conversion ratios for each type of encoder
    public static final double ENCODER_TICKS_TO_DEGREES = 1;
    public static final double ABSOLUTE_TICKS_TO_DEGREES = 1;
    
    //pid values for the motors
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kIz = 0;
    public static final double kFF = 0;

    //motors go in counterclockwise order starting with the front right motor as #1

    //these motors power the wheels
    public static final int DRIVE_ID_1 = 0;
    public static final int DRIVE_ID_2 = 0;
    public static final int DRIVE_ID_3 = 0;
    public static final int DRIVE_ID_4 = 0;

    //these motors spin the wheels to the correct angle
    public static final int TURN_ID_1 = 0;
    public static final int TURN_ID_2 = 0;
    public static final int TURN_ID_3 = 0;
    public static final int TURN_ID_4 = 0;

    //these absolute encoders remember the position of the wheels even when the robot is turned off
    public static final int ENCODER_1 = 0;
    public static final int ENCODER_2 = 0;
    public static final int ENCODER_3 = 0;
    public static final int ENCODER_4 = 0;


  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
