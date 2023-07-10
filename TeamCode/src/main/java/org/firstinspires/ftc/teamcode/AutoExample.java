package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Localizer;
import org.firstinspires.ftc.teamcode.PurePursuit.Mecanum;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;

@Autonomous(name = "Auto Example", group = "")
public class AutoExample extends OpMode {
    Mecanum drive;
    Localizer localizer;
    @Override
    public void init() {
        drive = new Mecanum(hardwareMap);
        localizer = new Localizer(hardwareMap, new Pose2D(0, 0, 0));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(true) {
            Pose2D target = new Pose2D(20, 20, 90);
            drive.auto(localizer.getPoseEstimate(), target, 1);
        }

        localizer.update();
    }

    @Override
    public void stop() {
    }

}