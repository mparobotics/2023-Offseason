package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.Config.SwerveModuleConstants;

public final class Constants {

    public static final class SwerveConstants{
        public static final double stickDeadband = 0.1;//stops the robot from driving when the stick isn't being pushed enough

        public static final int PIGEON_ID = 17;// placeholder
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

            /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;//sets how fast it accelerates 
        public static final double closedLoopRamp = 0.0;//same as open but feeds the output back into the loop

    
        public static final double driveGearRatio = (8.14 / 1.0); // L1: 8.14:1 L2: 6.75:1 L3: 6.12:1 L4: 5.14:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
            //translation2d is the translation of an object in a 2d space. this gives the location of the robot's swerve module in a 2d plane

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;//stops current from getting too high
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;//All of these PID values need tuning
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

         /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;//these also need changing
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

         /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;//finds out how far one motor spin goes
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;//how many degrees are in a motor spin

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

            /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

         /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false;//true if using MK4i

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(327.48046875);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
        }
  
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(286.34765625);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      } 

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.01953125);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      }
  
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 15;
            public static final int ANGLE_MOTOR_ID = 16;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.939453125);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      }
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}