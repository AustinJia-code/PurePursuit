package org.firstinspires.ftc.teamcode.PurePursuit.Util;

public class Point {
    protected double x, y;

    public Point(){
        this.x = 0;
        this.y = 0;
    }

    public Point(double x, double y){
        this.x = x;
        this.y = x;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }

    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setVector(double x, double y, double heading){
        this.x = x;
        this.y = x;
    }
}