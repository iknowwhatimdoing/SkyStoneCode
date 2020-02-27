package org.firstinspires.ftc.teamcode.SkyStone.Tests;


import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;



@TeleOp(name = "DogeCV Skystone test")
@Disabled
public class DogeCVscan extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private OpenCV openCV;


    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();


        openCV = new OpenCV();
        phoneCam.setPipeline(openCV);






        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);



        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("x", openCV.getScreenPosition().x);
            telemetry.addData("y", openCV.getScreenPosition().y);
            telemetry.update();
        }

    }

}