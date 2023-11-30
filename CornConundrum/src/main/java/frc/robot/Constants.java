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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class PneumaticsConstants{
    public static final int FORWARD_PORT = 0;
    public static final int REVERSE_PORT = 0;
    
  }
  public static class DriveConstants{
    public static final double ENCODER_VALUE_TO_ROTATIONS = -1.0 / 4096;
    public static final double ROTATIONS_TO_METERS = 0.1524 * Math.PI;
    public static final double ENCODER_TO_METERS = ENCODER_VALUE_TO_ROTATIONS * ROTATIONS_TO_METERS;
  }
}
