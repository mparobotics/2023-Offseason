// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class FancyJoystick extends Joystick{
    public FancyJoystick(int port){
       super(port);
    }
    
    public double getThrottle(){
        return this.getRawAxis(2);
    }
    public double getRotation(){
        return this.getRawAxis(5);
    }

    
}
