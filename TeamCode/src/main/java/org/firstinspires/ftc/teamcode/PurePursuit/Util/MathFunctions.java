package org.firstinspires.ftc.teamcode.PurePursuit.Util;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import java.util.ArrayList;

public class MathFunctions {
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePointOne, Point linePointTwo){

        // if x or y dimensions of points are close just approximate equality
        if(Math.abs(linePointOne.getY() - linePointTwo.getY()) < 0.003){
            linePointOne.setY(linePointTwo.getY() + 0.003);
        }
        if(Math.abs(linePointOne.getX() - linePointTwo.getX()) < 0.003){
            linePointOne.setX(linePointTwo.getX() + 0.003);
        }

        double m1 = (linePointTwo.getY() - linePointOne.getY()) / (linePointTwo.getX() - linePointOne.getX());

        double quadraticA = 1.0 + pow(m1, 2);
        double x1 = linePointOne.getX() - circleCenter.getX();
        double y1 = linePointOne.getY() - circleCenter.getY();

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1, 2) * x1);

        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2.0 * y1 * m1 * x1) + pow(y1 , 2) - pow(radius, 2);

        ArrayList<Point> intersections = new ArrayList<>();

        try{
            double sqrtDiscriminant = sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC));

            double xRoot1 = (-quadraticB + sqrtDiscriminant) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.getX();
            yRoot1 += circleCenter.getY();

            double minX = linePointOne.getX() < linePointTwo.getX() ? linePointOne.getX() : linePointTwo.getX();
            double maxX = linePointOne.getX() > linePointTwo.getX() ? linePointOne.getX() : linePointTwo.getX();

            if(xRoot1 > minX && xRoot1 < maxX){
                intersections.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrtDiscriminant) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.getX();
            yRoot2 += circleCenter.getY();

            if(xRoot2 > minX && xRoot2 < maxX){
                intersections.add(new Point(xRoot2, yRoot2));
            }
        }catch(Exception E){
            System.out.println("No roots");
        }

        return intersections;
    }

    public static double AngleWrapRad(double angle){
        while(angle < -Math.PI) angle += 2*Math.PI;
        while(angle > Math.PI) angle -= 2*Math.PI;

        return angle;
    }

    public static double AngleWrapDeg(double angle){
        return Math.toDegrees(AngleWrapRad(Math.toRadians(angle)));
    }
}
