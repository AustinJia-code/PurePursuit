package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Constants.Mecanum.SENSITIVITY_IN;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.PurePursuit.Util.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.MathFunctions;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.PID;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Point;

import java.util.ArrayList;

public class Mecanum implements Subsystem {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;
    PID xController, yController, headingController;
    boolean atTarget;
    Pose2D currentPose, targetPose;
    private RevIMU imu;
    private Mode mode;
    private double speed = 0.85;
    enum Mode{FIELD, ROBOT}
    public Mecanum(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = new RevIMU(hardwareMap);
        imu.init();
        mode = Mode.FIELD;
    }
    public void initAuto(){
        xController = new PID(8.25, 0.25, 0);
        yController = new PID(8.25, 0.25, 0);
        headingController = new PID(8.25, 0.25, 0);
    }
    public void setMode(Mode m){
        mode = m;
    }
    public void recenter(){imu.reset();}
    public void switchModes(){
        if(mode.equals(Mode.ROBOT)){
            mode = Mode.FIELD;
        }else{
            mode = Mode.ROBOT;
        }
    }
    public String getMode(){
        if(mode == Mode.FIELD){
            return "FIELD CENTRIC";
        }else{
            return "ROBOT CENTRIC";
        }
    }
    public void teleOp(GamepadEx gamepad){
        y = Math.pow(gamepad.getLeftY(), 3);
        x = Math.pow(gamepad.getLeftX()*1.1, 3);
        rx = Math.pow(gamepad.getRightX(), 3);

        switch (mode){
            case FIELD:
                heading = Math.toRadians(-imu.getHeading()+180);
                rotX = x * Math.cos(heading) - y * Math.sin(heading);
                rotY = x * Math.sin(heading) + y * Math.cos(heading);

                drive(rotX, rotY, rx);
                break;
            case ROBOT:
                drive(x, y, rx);
                break;
        }
    }
    public void autoMimicPath(Pose2D targetPose, double speed, double turnSpeed){
        double headingError, xError, yError, sens, lateralError;
        double lateralTolerance = 0.5, headingTolerance = 2.0 / 180;
        this.targetPose = targetPose;

        lateralError = Math.sqrt(
                Math.pow((currentPose.getX() - targetPose.getX()), 2)
                +
                Math.pow((currentPose.getY() - targetPose.getY()), 2));

        headingError = (currentPose.getHeading() - targetPose.getHeading()) % 360;
        xError = (lateralError * Math.cos(Math.toRadians(headingError))) / SENSITIVITY_IN;
        yError = (lateralError * Math.sin(Math.toRadians(headingError))) / SENSITIVITY_IN;
        headingError /= 180;

        if(xError * SENSITIVITY_IN < lateralTolerance && yError * SENSITIVITY_IN < lateralTolerance && headingError < headingTolerance){
            xController.reset();
            yController.reset();
            headingController.reset();
            atTarget = true;
        }else {
            xController.calculate(xError);
            yController.calculate(yError);
            headingController.calculate(headingError);
            atTarget = false;
        }

        drive(xController.getOutput() * speed, yController.getOutput() * speed, headingController.getOutput() * turnSpeed);
    }

    public void autoToPoint(Point targetPose, double speed, double turnSpeed){
        double targetHeading = Math.atan((targetPose.getX() - currentPose.getX()) / (targetPose.getY() - currentPose.getY()));

        Pose2D newTarget = new Pose2D(targetPose.getX(), targetPose.getY(), targetHeading);

        autoMimicPath(newTarget, speed, turnSpeed);
    }

    private void drive(double x, double y, double rx){
        leftFrontPower = (y + x + rx);
        leftRearPower = (y - x + rx);
        rightFrontPower = (y - x - rx);
        rightRearPower = (y + x - rx);

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint((pathPoints.get(0)));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = Double.MAX_VALUE;

            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.getY() - y, thisIntersection.getX() - x);
                double deltaAngle = Math.abs(MathFunctions.AngleWrapRad(angle - Math.toRadians(heading)));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }
    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(x, y), allPoints.get(0).followDistance);

        autoToPoint(followMe.toPoint(), followMe.moveSpeed, followMe.turnSpeed);
    }
    public double getHeading(){
        return imu.getHeading();
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }
    public boolean atTarget(){ return atTarget; }
    public boolean atTarget(Pose2D target){
        double headingError, xError, yError, lateralError;
        double lateralTolerance = 0.5, headingTolerance = 2.0 / 180;

        lateralError = Math.sqrt(
                Math.pow((currentPose.getX() - target.getX()), 2)
                        +
                        Math.pow((currentPose.getY() - target.getY()), 2));

        headingError = (currentPose.getHeading() - target.getHeading()) % 360;
        xError = (lateralError * Math.cos(Math.toRadians(headingError))) / SENSITIVITY_IN;
        yError = (lateralError * Math.sin(Math.toRadians(headingError))) / SENSITIVITY_IN;
        headingError /= 180;

        if(xError * SENSITIVITY_IN < lateralTolerance && yError * SENSITIVITY_IN < lateralTolerance && headingError < headingTolerance){
            return true;
        }
        return false;
    }

    public void setPoseEstimate(Pose2D poseEstimate){
        currentPose.setPose(poseEstimate);
    }
}