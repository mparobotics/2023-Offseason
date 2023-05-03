// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDsubsystem extends SubsystemBase {
  /** Creates a new LEDsubsystem. */
  private CANdle leds = new CANdle(LEDConstants.CANDLE_ID);
  public LEDsubsystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    leds.configAllSettings(configAll, 100);

  }
  public void setColor(int[] color){
    leds.setLEDs(color[0], color[1], color[2],0,0,LEDConstants.LED_COUNT);
  }
  public CommandBase Cube(){
    System.out.println("changed Leds to purple");
    return runOnce(() -> setColor(LEDConstants.PURPLE_RGB));
    
  }
  public CommandBase Cone(){
    return runOnce(() -> setColor(LEDConstants.YELLOW_RGB));
  }
  @Override
  public void periodic() {
    //Cube();
    // This method will be called once per scheduler run
  }
}
