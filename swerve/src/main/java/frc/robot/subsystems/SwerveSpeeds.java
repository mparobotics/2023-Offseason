// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



/** Add your docs here. */
public class SwerveSpeeds {
    //speed in (x,y) format
    public double x = 0;
    public double y = 0;
    
    //speed in direction, magnitude format
    public double speed = 0;
    public double angle = 0;
    //absolute value of speed
    public double absSpeed = 0;
    //create a SwerveSpeeds with x and y
    public static SwerveSpeeds XY(double x, double y){
        return new SwerveSpeeds(x, y, false);
    }
    //create a SwerveSpeeds with speed and direction
    public static SwerveSpeeds SD(double angle, double speed){
        return new SwerveSpeeds(angle, speed, true);
    }
    //scale the speed by a given value
    public void scaleBy(double value){
        this.speed *= value;
        this.absSpeed *= value;

        this.x *= value;
        this.y *= value;
    }
    private SwerveSpeeds(double a,double b, boolean isPolar){
        if(isPolar){
            //set speed and diection using inputs
            this.speed = a;
            this.angle = b;
            
            //calculate x and y using speed and direction
            this.x = Math.cos(b) * a;
            this.y = Math.sin(b) * a;
        }
        else{
            //set x and y using inputs
            this.x = a;
            this.y = b;

            //calculate speed and angle using x and y
            this.speed = Math.hypot(a,b);
            this.angle = Math.atan2(b,a);
        }
        //set absolute value of speed
        this.absSpeed = Math.abs(speed);
    }
    
}
