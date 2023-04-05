// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



/** A 2d coordinate class - useful for converting from xy coordinates to polar coordinates*/
public class Coord2D {
    //speed in (x,y) format
    private double x = 0;
    private double y = 0;
    
    //speed in (direction, length) format
    private double length = 0;
    private double direction = 0;



    //create a coord2d with x and y
    public static Coord2D XY(double x, double y){
        return new Coord2D(x, y, false);
    }
    //create a coord2d with speed and direction
    public static Coord2D SD(double length, double direction){
        return new Coord2D(length, direction, true);
    }
    //scale the speed by a given value
    public void scaleBy(double value){
        length *= value;

        x *= value;
        y *= value;
    }
    private Coord2D(double a,double b, boolean isPolar){
        if(isPolar){
            //set speed and diection using inputs
            this.length = a;
            this.direction = b;
            
            //calculate x and y using speed and direction
            this.x = Math.cos(Math.toRadians(b)) * a;
            this.y = Math.sin(Math.toRadians(b)) * a;
        }
        else{
            //set x and y using inputs
            this.x = a;
            this.y = b;

            //calculate speed and angle using x and y
            this.length = Math.hypot(a,b);
            this.direction = Math.toDegrees(Math.atan2(b,a));
        }
    }
    public double length(){
        return length;
    }
    public double absLength(){
        return Math.abs(length);
    }
    public double direction(){
        return direction;
    }
    public double x(){
        return x;
    }
    public double y(){
        return y;
    }

}
