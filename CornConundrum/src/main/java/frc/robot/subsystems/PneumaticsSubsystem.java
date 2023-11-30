// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticsSubsystem extends SubsystemBase {
  DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticsConstants.FORWARD_PORT, PneumaticsConstants.REVERSE_PORT);

  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase out() {
    return runOnce(() -> doubleSolenoid.set(Value.kForward));
  }

  public CommandBase in() {
    return runOnce(() -> doubleSolenoid.set(Value.kReverse));
  }

  public CommandBase toggle() {
    return runOnce(() -> doubleSolenoid.toggle());
  }
}
