package org.firstinspires.ftc.teamcode.PurePursuit.Util;

public class Pose2D {
    private double x, y, heading;
    public Pose2D(){
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }
    public Pose2D(double x, double y, double heading){
        this.x = x;
        this.y = x;
        this.heading = heading;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }
    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setHeading(){
        this.heading = heading;
    }
    public void setPose(double x, double y, double heading){
        this.x = x;
        this.y = x;
        this.heading = heading;
    }
}
