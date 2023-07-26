package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Constants.Localizer.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.PurePursuit.Constants.Localizer.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.PurePursuit.Constants.Localizer.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.PurePursuit.Constants.Localizer.WHEEL_DIAMETER_IN;

import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;

//implementation of https://gm0.org/en/latest/docs/software/concepts/odometry.html

public class Localizer {
    private Pose2D poseEstimate;
    private Encoder leftEncoder, perpendicularEncoder, rightEncoder;

    private double leftCurrent, perpendicularCurrent, rightCurrent;
    private double leftPrevious, perpendicularPrevious, rightPrevious;
    private double leftDelta, perpendicularDelta, rightDelta;

    private double thetaDelta, xPosition, yPosition, heading, middleDelta, perpDelta, xDelta, yDelta;

    public Localizer(HardwareMap hardwareMap, Pose2D poseEstimate){
        leftEncoder = hardwareMap.get(Encoder.class, "leftEncoder");
        rightEncoder = hardwareMap.get(Encoder.class, "rightEncoder");
        perpendicularEncoder = hardwareMap.get(Encoder.class, "perpendicularEncoder");

        this.poseEstimate = poseEstimate;
    }
    public Pose2D getPoseEstimate(){
        return poseEstimate;
    }

    public void update(){
        leftCurrent = leftEncoder.getPosition(); perpendicularCurrent = perpendicularEncoder.getPosition(); rightCurrent = rightEncoder.getPosition();

        leftDelta = leftCurrent - leftPrevious;
        perpendicularDelta = perpendicularCurrent - perpendicularPrevious;
        rightDelta = rightCurrent - rightPrevious;

        thetaDelta = (leftDelta - rightDelta) / TRACKWIDTH;
        middleDelta = (leftDelta + rightDelta) / 2;
        perpDelta = perpendicularDelta - FORWARD_OFFSET * thetaDelta;

        xDelta = (middleDelta * Math.cos(heading) - perpDelta * Math.sin(heading)) / ENCODER_RESOLUTION * (WHEEL_DIAMETER_IN * Math.PI);
        yDelta = middleDelta * Math.sin(heading) - perpDelta * Math.cos(heading) / ENCODER_RESOLUTION * (WHEEL_DIAMETER_IN * Math.PI);

        xPosition += xDelta;
        yPosition += yDelta;
        heading += thetaDelta;

        leftPrevious = leftCurrent; perpendicularPrevious = perpendicularCurrent; rightPrevious = rightCurrent;

        poseEstimate.setPose(xPosition, yPosition, heading);
    }

    public double getxPosition() {
        return xPosition;
    }

    public double getyPosition() {
        return yPosition;
    }

    public double getHeading() {
        return heading;
    }
}
