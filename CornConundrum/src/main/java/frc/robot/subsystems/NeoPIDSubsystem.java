// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoPIDSubsystem extends SubsystemBase {
  /** Creates a new PIDSubsystem. */
  private final int NEO_ID = 0;
  private CANSparkMax test_neo = new CANSparkMax(NEO_ID, MotorType.kBrushless);
  private SparkMaxPIDController pid = test_neo.getPIDController();

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  public NeoPIDSubsystem() {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }
  public void setNEOSetpoint(double position){
    pid.setReference(position,ControlType.kPosition);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
