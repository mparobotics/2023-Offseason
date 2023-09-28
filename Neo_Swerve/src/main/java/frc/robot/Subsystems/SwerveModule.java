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
import frc.lib.Config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState; // Libraries I'll do later
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
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
}
