package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Localizer;
import org.firstinspires.ftc.teamcode.PurePursuit.Mecanum;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;

import java.util.ArrayList;

@Autonomous(name = "Auto Example", group = "")
public class AutoExample extends OpMode {
    Mecanum drive;
    Localizer localizer;
    ArrayList<CurvePoint> allPoints;
    @Override
    public void init() {
        drive = new Mecanum(hardwareMap);
        localizer = new Localizer(hardwareMap, new Pose2D(0, 0, 0));

        allPoints = new ArrayList<>();

        allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        allPoints.add(new CurvePoint(180, 180, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        allPoints.add(new CurvePoint(220, 180, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        allPoints.add(new CurvePoint(280, 50, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        drive.followCurve(allPoints, Math.toRadians(90));

        localizer.update();
        drive.setPoseEstimate(localizer.getPoseEstimate());
    }

    @Override
    public void stop() {
    }

}