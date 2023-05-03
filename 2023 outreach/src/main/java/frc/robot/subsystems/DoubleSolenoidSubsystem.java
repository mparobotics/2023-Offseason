// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DoubleSolenoid;



public class DoubleSolenoidSubsystem extends SubsystemBase {                                                  
  /** Creates a new DoubleSolenoidSubsystem. */
  DoubleSolenoid doubleSolenoidShort = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5); // makes a new double solenoid class at 1(retracts) and 0(forward)
  DoubleSolenoid doubleSolenoidLong = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 3);


  public DoubleSolenoidSubsystem() {
    //enables the compressor
    //boolean isEnabled = phCompressor.isEnabled(); //boolean = true or false, "isEnabled" holds either true or false value depending on if the compressor is enabled or not
    //boolean pressureSwitch = phCompressor.getPressureSwitchValue(); //again, "pressureSwitch" holds either true or false depending if there is pressure applied to pressure switch or not
    //Shoot = long retracted, short extended 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public CommandBase retract() //if forwardSolenoid is called, returns a command to set doublesolenoid forward at port 0
  {
    System.out.println("Retracting!");
    return runOnce(
      () -> {
        doubleSolenoidShort.set(Value.kReverse); //O
        doubleSolenoidLong.set(Value.kReverse); //O
      });
  }
  
  public CommandBase chuteintake() //if reverseSolenoid is called, returns a command to set doublesolenoid reverse at port 1
  {
    System.out.println("Chute Intake!");
    return runOnce(
      () -> {
        doubleSolenoidShort.set(Value.kForward); //X
        doubleSolenoidLong.set(Value.kReverse); //O
      });
  }



  public CommandBase shoot()
  {
    System.out.println("Shooting!");
    return runOnce(
      () -> {
        doubleSolenoidShort.set(Value.kReverse); //O
        doubleSolenoidLong.set(Value.kForward); //X
      });
  }

  public CommandBase groundintake()
  {
    System.out.println("GroundIntake!");
    return runOnce(
      () -> {
        doubleSolenoidShort.set(Value.kForward); //X
        doubleSolenoidLong.set(Value.kForward); //X
      });
  }
}
