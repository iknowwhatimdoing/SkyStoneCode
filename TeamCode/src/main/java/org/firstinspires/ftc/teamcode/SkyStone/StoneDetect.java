package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Stone Detect")
//@Disabled
public class StoneDetect extends LinearOpMode {

    SkyStoneHardware robot   = new SkyStoneHardware();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    private static final String VUFORIA_KEY =
            "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime time = new ElapsedTime();





    double K_P = 0;
    double K_I = 0;
    double K_D = 0;




    @Override
    public void runOpMode() {


        robot.init(hardwareMap);



        robot.modernRoboticsI2cGyro.calibrate();


        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("Gryo: ", "Calibrating");
            telemetry.update();
        }

        telemetry.addData("Gryo: ","Calibrated");
        telemetry.update();


        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();



        scan();






    }











    /**     Methods
     ----------------------------------------------------------------------------------
     **/






    private void move(double targetpos){

        double intergal = 0;
        double iterations = 0;
        ElapsedTime timer = new ElapsedTime();
        double targetPosition = targetpos;

        double error = robot.leftfront.getCurrentPosition() - targetPosition;
        double lastError = 0;
        double kp = 0.01;
        double ki = 0.008;
        double kd = 0.008;

        while (Math.abs(error) <= 5){

            error = robot.leftfront.getCurrentPosition() - targetPosition;
            double deltaError = lastError - error;
            intergal += deltaError * timer.time();
            double derivative = deltaError / timer.time();

            double power = (kp * error) + (ki * intergal) + (kd * derivative);


            if(targetpos > 0) {
                robot.driveAll(Range.clip(power,-1,1));
            }else if(targetpos < 0) {
                robot.driveAll(Range.clip(-power,-1,1));
            }


            error = lastError;
            iterations++;
            timer.reset();
        }



        robot.driveAll(0);


    }


    private void turn(double desiredturn){


        int heading = robot.modernRoboticsI2cGyro.getHeading();
        int integratedZ;

        double error = desiredturn - heading;


        while (opModeIsActive() && (error > 1 || error < 1))  {



            heading = robot.modernRoboticsI2cGyro.getHeading();
            integratedZ = robot.modernRoboticsI2cGyro.getIntegratedZValue();

            float zAngle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;



            telemetry.addData("angle", "%s deg", String.format("%.3f",zAngle));
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("integrated Z", "%3d", integratedZ);
            telemetry.update();
        }
    }




    private void collectStone(String location){

        switch(location){

            case "left":

                //motions to collect stone on the left

                break;

            case "middle":

                //motions to collect stone in the middle

                break;

            case "right":

                //motions to collect stone on the right

                break;

        }
    }






    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    private void scan(){


        boolean stoneL = false;
        boolean stoneM = false;
        boolean stoneR = false;


        if (opModeIsActive()) {


            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();


                if (updatedRecognitions != null && updatedRecognitions.size() >= 3) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());


                        if (recognition.getLabel() == LABEL_SECOND_ELEMENT){

                            if (0.5 <= recognition.getLeft() && recognition.getLeft() <= 1.0){
                                stoneL = true;
                            }else if (1.1 <= recognition.getLeft() && recognition.getLeft() <= 2.0){
                                stoneM = true;
                            }else if (2.1 <= recognition.getLeft() && recognition.getLeft() <= 3.0){
                                stoneR = true;
                            }


                        }

                    }

                    telemetry.update();
                }
            }





        }

        if (tfod != null) {
            tfod.shutdown();
        }


        collectionOrder();
    }

    private void collectionOrder(){



    }


    /*
    boolean sl = true;
        boolean sm = true;
        boolean sr = true;

        //plan path
        while (opModeIsActive() && (stoneL == true || stoneM == true || stoneR == true)) {
            if (stoneL) {

                collectStone("left");
                stoneL = false;
                sl = false;

            } else if (stoneM) {

                collectStone("middle");
                stoneM = false;
                sm = false;

            } else if (stoneR) {

                collectStone("right");
                stoneR = false;
                sr = false;

            } else {

                //all skystones are collected

            }
        }



        while (opModeIsActive() && (sl == true || sm == true || sr == true)) {

            if (sl) {

                collectStone("left");
                sl = false;

            } else if (sm) {

                collectStone("middle");
                sm = false;

            } else if (sr) {

                collectStone("right");
                sr = false;

            } else {

                //all stones have been collected

            }



        }

     */
}
