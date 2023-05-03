// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase 
{


  /** Creates a new IntakeSubsystem. */
  // creating motors
  private final CANSparkMax intakeMotorR = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_R_ID, MotorType.kBrushless);
  private final CANSparkMax intakeMotorL = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_L_ID, MotorType.kBrushless);
  public RelativeEncoder intakeEncoderL = intakeMotorL.getEncoder();
  public RelativeEncoder intakeEncoderR = intakeMotorR.getEncoder();
  public double intakeTestSpeed = .5;
  

  //Inverting left motor so wheels spin in same direction 
  public IntakeSubsystem()
  {
  SmartDashboard.putNumber("Intake Speed", intakeTestSpeed);  
  
  intakeMotorR.setInverted(true);
  intakeMotorL.setInverted(false);

  intakeMotorL.setSmartCurrentLimit(40, 200);
  intakeMotorR.setSmartCurrentLimit(40, 200);

  

  intakeMotorL.setIdleMode(IdleMode.kBrake);
  intakeMotorR.setIdleMode(IdleMode.kBrake);
  }
  
  //setting speed of motors for intake
  public void set(double speed)
  {
      /* one-time action goes here*/
      intakeMotorL.set(speed);
      intakeMotorR.set(speed);
  }
  //setting speed of motors for outtake
  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotor L", intakeEncoderL.getVelocity());
    SmartDashboard.putNumber("intakeMotor R", intakeEncoderR.getVelocity());
    // This method will be called once per scheduler run
    double is = SmartDashboard.getNumber("Intake Speed", .5);
    if((is != intakeTestSpeed)) { intakeTestSpeed = is; }
  }
}
