package org.firstinspires.ftc.teamcode.SkyStone.Tests;


import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@TeleOp(name = "DogeCV Skystone test (not mine)")
@Disabled
public class DogeCVscan2 extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private SkystoneDetector skystoneDetector;


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();


        skystoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skystoneDetector);





        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);



        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Left Point Pos", skystoneDetector.getScreenPosition().x);
            telemetry.update();
        }

    }

}