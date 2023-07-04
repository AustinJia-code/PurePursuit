package org.firstinspires.ftc.teamcode.PurePursuit;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.*;

public class Mecanum implements Subsystem {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;
    PID xController, yController, headingController;
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

                leftFrontPower = (rotY + rotX + rx);
                leftRearPower = (rotY - rotX + rx);
                rightFrontPower = (rotY - rotX - rx);
                rightRearPower = (rotY + rotX - rx);
                break;
            case ROBOT:
                leftFrontPower = (y + x + rx);
                leftRearPower = (y - x + rx);
                rightFrontPower = (y - x - rx);
                rightRearPower = (y + x - rx);
                break;
        }
        drive(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }
    public void auto(Pose2D targetPose, double speed, double sensitivity){
        double headingError, xError, yError, sens;
        double lateralTolerance = 0.5, headingTolerance = 2;
        this.targetPose = targetPose;
        this.speed = speed;
        sens = 12;

        xError = (currentPose.getX() - targetPose.getX()) / sens;
        yError = (currentPose.getY() - targetPose.getY()) / sens;
        headingError = (currentPose.getHeading() - targetPose.getHeading()) % 360;

        if(xError * sens < lateralTolerance && yError * sens < lateralTolerance && headingError < headingTolerance){
            xController.reset();
            yController.reset();
            headingController.reset();
        }else {
            xController.calculate(xError);
            yController.calculate(yError);
            headingController.calculate(headingError);
        }

        drive(xController.getOutput(), yController.getOutput(), headingController.getOutput(), speed);  
    }
    private void drive(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower){
        leftFront.setPower(leftFrontPower * speed);
        leftRear.setPower(leftRearPower * speed);
        rightFront.setPower(rightFrontPower * speed);
        rightRear.setPower(rightRearPower * speed);
    }
    public double getHeading(){
        return imu.getHeading();
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }
}