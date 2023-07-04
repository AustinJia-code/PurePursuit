package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Multithread Example", group = "")
public class MultithreadExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        Thread thread = new Thread(new exampleThread());
        while (!isStarted() && !isStopRequested()) {
            //post init loop
        }
        if (opModeIsActive()) {
            //start
            thread.start();
            while (opModeIsActive()) {
                //end loop
            }
        }

    }
    class exampleThread implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive() && !isStopRequested()) {
                //Multithreaded
            }
        }
    }

}