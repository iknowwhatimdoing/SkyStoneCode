//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "Quarry Side (Odometry)")
public class Quarry_Side_Odometry extends LinearOpMode {


    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;


    private String configuration = "Not Decided";
    private boolean stonesLeft = true;


    @Override
    public void runOpMode() {

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();



        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        //globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        //Thread positionThread = new Thread(globalPositionUpdate);
        //positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseNormalEncoder();


        //drive to where you can see the stones
        //moveDistancePID(.5,20);
        //goToPosition(1,1,.5,0,.5);

        //scan the stones
        scanCV();


        //collect the stones
        boolean skyStoneCollected = false;
        boolean leftCollected = false;
        boolean middleCollected = false;
        boolean rightCollected = false;

        while (opModeIsActive() && stonesLeft == true) {

            if (!skyStoneCollected) {
                if (configuration == "A") {
                    grabStone("left");
                    leftCollected = true;
                    skyStoneCollected = true;
                } else if (configuration == "B") {
                    grabStone("middle");
                    middleCollected = true;
                    skyStoneCollected = true;
                } else if (configuration == "C") {
                    grabStone("right");
                    skyStoneCollected = true;
                    rightCollected = true;
                }
            } else {

                if (!leftCollected) {
                    grabStone("left");
                    leftCollected = true;
                } else if (!middleCollected) {
                    grabStone("middle");
                    middleCollected = true;
                } else if (!rightCollected) {
                    grabStone("right");
                    rightCollected = true;
                } else {
                    stonesLeft = false;
                }

            }

            if (stonesLeft == true) {
                placeOnFoundation();

                returnToHomingPosition();
            }

        }

        //park on middle line
        //drive from the homing position


        //globalPositionUpdate.stop();


    }







    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/

/*
    public void moveDistancePID(double speed, double inches) {


        double targetPosition = (int) (inches * COUNTS_PER_INCH);
        double intergral = 0;
        double iteratoins = 0;
        ElapsedTime timer = new ElapsedTime();

        double error = robot.leftfront.getCurrentPosition() - targetPosition;
        double lastError = 0;
        double Kp = 0.01;
        double Ki = 0.008;
        double Kd = 0.008;

        while (opModeIsActive() && Math.abs(error) <= 5) {
            error = robot.leftfront.getCurrentPosition() - targetPosition;
            double deltaError = lastError - error;
            intergral += deltaError * timer.time();
            double derivative = deltaError / timer.time();
            robot.driveAll(Range.clip(Kp * error + Ki * intergral + Kd + derivative,-speed,speed));
            error = lastError;
            iteratoins++;
            timer.reset();

        }


    }

 */


    public void goToPosition(double targetXPos, double targetYPos, double power, double desiredRobotOrientation, double allowedDistanceError) {
        targetXPos *= COUNTS_PER_INCH;
        targetYPos *= COUNTS_PER_INCH;
        allowedDistanceError *= COUNTS_PER_INCH;

        double distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while (opModeIsActive() && distance > allowedDistanceError) {


            distanceToXTarget = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPos - globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            //may need to flip the signs for the pivotCorrection
            left_front.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            left_back.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent + pivotCorrection), -1, 1));
            right_front.setPower(Range.clip((-robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
            right_back.setPower(Range.clip((robotMovementXComponent + robotMovementYComponent - pivotCorrection), -1, 1));
        }
    }


    private void scanCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        CameraDevice.getInstance().setFlashTorchMode(true);
        phoneCam.openCameraDevice();


        skyStoneDetector = new SkystoneDetector();

        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while (opModeIsActive() && configuration == "Not Decided") {



            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            telemetry.update();

            String filename = "X_Positions.txt";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(skyStoneDetector.getScreenPosition().x));


        }
        phoneCam.stopStreaming();

    }




    private void grabStone(String position) {


        if (position == "left") {
            //drive sideways left
        } else if (position == "middle") {
            //drive sideways to align
        } else if (position == "right") {
            //drive sideways right
        }

        //drive forward
        //moveDistancePID(.5, 30);

        // grab it

        //back up
        //moveDistancePID(.5,-30);

        if (position == "left") {
            //drive back
        } else if (position == "middle") {
            //drive back
        } else if (position == "right") {
            //drive back
        }

    }


    private void placeOnFoundation() {
        //back up more if needed

        //turn based on witch side your on
        //use distance sensor to see witch team your on an turn accordingly

        //drive forward until reach the foundation
        //wait till safe auto and test to find how to tell where the foundation is

        //turn by how you need to

        //drive forward

        //drop

        //dive back stuff
    }


    private void returnToHomingPosition() {
        //maybe use vuforia to line self up with target
        //add stuff late if needed
    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}
