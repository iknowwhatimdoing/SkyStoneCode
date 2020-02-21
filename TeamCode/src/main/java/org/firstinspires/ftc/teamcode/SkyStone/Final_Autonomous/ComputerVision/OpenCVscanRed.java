package org.firstinspires.ftc.teamcode.SkyStone.Final_Autonomous.ComputerVision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVscanRed extends OpenCvPipeline {

    public String position = "";

    Mat original = new Mat();
    Mat workingMat = new Mat();
    Mat display = new Mat();



    public OpenCVscanRed(){
    }


    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(original);
        input.copyTo(workingMat);
        input.copyTo(display);


        if (workingMat.empty()){
            return input;
        }

        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb);

        Mat leftMat = workingMat.submat(120,150,35,75);
        Mat midMat = workingMat.submat(120,150,105,145);
        Mat rightMat = workingMat.submat(120,150,175,215);

        Imgproc.rectangle(display, new Rect(35, 120, 40 , 30), new Scalar(0,0,255), 2);
        Imgproc.rectangle(display, new Rect(105, 120, 40 , 30), new Scalar(0,0,255),2);
        Imgproc.rectangle(display, new Rect(175, 120, 40 , 30), new Scalar(0,0,255),2);


        double leftScore = Core.sumElems(leftMat).val[2];
        double midScore = Core.sumElems(midMat).val[2];
        double rightScore = Core.sumElems(rightMat).val[2];

        if (leftScore > midScore){
            if (leftScore > rightScore){
                position = "LEFT";
            }else{
                position = "RIGHT";
            }
        }else{
            if (midScore > rightScore){
                position = "MIDDLE";
            }else{
                position = "RIGHT";
            }
        }

        return display;
    }
}
