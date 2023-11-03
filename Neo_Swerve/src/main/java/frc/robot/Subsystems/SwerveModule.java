// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Config.SwerveModuleConstants;
import frc.lib.Config.OnboardModuleState;
import frc.lib.Config.CANCoderUtil;
import frc.lib.Config.CANCoderUtil.CANCoderUsage;
import frc.lib.Config.CANSparkMaxUtil;
import frc.lib.Config.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;
/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;
 
    private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
        Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

          /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState(){
      return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
      desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

      setAngle(desiredState);
      setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        //when not taking feedback
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
        driveMotor.set(percentOutput);
      } 
      else {
        driveController.setReference(
          desiredState.speedMetersPerSecond, 
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
      }
    }

    private void setAngle(SwerveModuleState desiredState) {
      //Prevent rotating module if speed is less then 1%. Prevents Jittering.
      Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;

      angleController.setReference(angle.getDegrees(), ControlType.kPosition);
      lastAngle = angle;
    }


    private void resetToAbsolute() {
      double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
      integratedAngleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getCanCoder() {
      return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private void configAngleEncoder() {
      angleEncoder.configFactoryDefault();
      CANCoderUtil.setCANCoderBusUsage(angleEncoder, CANCoderUsage.kMinimal);
      angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfiguration);
    }

    private void configAngleMotor(){
      angleMotor.restoreFactoryDefaults(); //resets the motor
      CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly); //limits can bus usage
      angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit); //limits current
      angleMotor.setInverted(Constants.SwerveConstants.angleInvert); //inverts it if needed
      angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode); //brakes
      integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor); 
      //sets a conversion factor for the encoder so it output correlates with the rotation of the module

      //PID
      angleController.setP(m_angleKP);
      angleController.setI(m_angleKI);
      angleController.setD(m_angleKD);
      angleController.setFF(m_angleKFF);
      angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
      angleMotor.burnFlash();  //BURN THE SPARK MAX

      Timer.delay(1.0);
      resetToAbsolute();// reset
    }

    private void configDriveMotor(){
      driveMotor.restoreFactoryDefaults(); //resets the motor
      CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll); //UNLIMITED CAN BUS USAGE
      driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit); //limits current
      driveMotor.setInverted(Constants.SwerveConstants.driveInvert); //inverts it if needed
      driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode); //brakes
     

    }
}
