package org.firstinspires.ftc.teamcode.PurePursuit;

import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;


public class Localizer {
    private Pose2D poseEstimate;
    private Encoder leftEncoder, perpendicularEncoder, rightEncoder;

    public Localizer(HardwareMap hardwareMap){
        leftEncoder = hardwareMap.get(Encoder.class, "leftEncoder");
        rightEncoder = hardwareMap.get(Encoder.class, "rightEncoder");
        perpendicularEncoder = hardwareMap.get(Encoder.class, "perpendicularEncoder");
    }
    public Pose2D getPoseEstimate(){
        return poseEstimate;
    }

    public void update(){

    }
}
