package org.firstinspires.ftc.teamcode.SkyStone.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

public class OpenCVscan extends OpMode {

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    Mat workingMat = new Mat();
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
