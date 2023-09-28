package frc.lib.Config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int ANGLE_MOTOR_ID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKFF;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param DRIVE_MOTOR_ID
   * @param ANGLE_MOTOR_ID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleConstants(
    int DRIVE_MOTOR_ID, int ANGLE_MOTOR_ID, int canCoderID, Rotation2d angleOffset, 
    double angleKP, double angleKI, double angleKD, double angleFF) {
  this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
  this.ANGLE_MOTOR_ID = ANGLE_MOTOR_ID;
  this.cancoderID = canCoderID;
  this.angleOffset = angleOffset;
  this.angleKP = angleKP;
  this.angleKI = angleKI;
  this.angleKD = angleKD;
  this.angleKFF = angleFF;
}
}