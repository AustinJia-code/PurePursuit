package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Localizer;
import org.firstinspires.ftc.teamcode.PurePursuit.Mecanum;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Point;
import org.firstinspires.ftc.teamcode.PurePursuit.Util.Pose2D;

import java.util.ArrayList;

@Autonomous(name = "Auto Example", group = "")
public class AutoExample extends OpMode {
    Mecanum drive;
    Localizer localizer;
    ArrayList<CurvePoint> curve1;
    Point Curve1End;
    @Override
    public void init() {
        drive = new Mecanum(hardwareMap);
        localizer = new Localizer(hardwareMap, new Pose2D(0, 0, 0));

        curve1 = new ArrayList<>();

        curve1.add(new CurvePoint(0, 0, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        curve1.add(new CurvePoint(180, 180, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        curve1.add(new CurvePoint(220, 180, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        curve1.add(new CurvePoint(280, 50, 1.0, 1.0, 90, Math.toRadians(90), 1.0, 1.0));
        Curve1End = curve1.get(curve1.size()-1).toPoint();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(drive.atTarget(new Pose2D(Curve1End.getX(), Curve1End.getY(), drive.getHeading()))) {
            telemetry.addData("at: ", "target");
        }else{
            drive.followCurve(curve1, Math.toRadians(90));
        }

        localizer.update();
        drive.setPoseEstimate(localizer.getPoseEstimate());

    }

    @Override
    public void stop() {
    }

}