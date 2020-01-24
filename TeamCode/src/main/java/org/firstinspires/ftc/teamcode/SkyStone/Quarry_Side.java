//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.SkyStone.Odometry.OdometryGlobalCoordinatePosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.File;
import java.security.Policy;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Quarry Side")
public class Quarry_Side extends LinearOpMode {

    boolean posFound = false;
    String pos = "";


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
            "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float stoneZ = 2.00f * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;




    /*
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AeVN5hL/////AAABmeiiuuNfCUG7o7nFHIB8wOIJd0HGVkf4o9TQNISpW19DjyAqW6a1+DIXgQEWJlZoXbHJhjRNbpdhsZiyF8YZS7mSkspYXxB9vhRl3NdBzba6lb450R331LCujbFW4f1z5ZiERvZsxUn1bl8FeugGjQAag3tO7DU7ZNFMRVev0pTtIo0tIRtVlW1zmZqCAQCmCLtAUPjpUv2atadLfVqleYVydV3AN2upMHu14tb3zBuOmM2SfHu//Nuo8xh/U2a0Joi9B286UYurYN7S/+2mzpe6cFcqdDjVV7C3sjvHUw3ivtGy9M7BXwsVZkF/aSQr0cjDfTv/si4DW8lm/WLG5thfi2kHv/B21I2C67dt/oGI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
     */


    //public DistanceSensor rightSideDist;
    //public DistanceSensor leftSideDist;
    //public DistanceSensor frontLeftDist;
    //public DistanceSensor frontRightDist;



    Servo odometryServo;
    Servo capstone;


    Servo frontClaw;
    DcMotor flipBackLeft;
    DcMotor flipBackRight;

    DcMotor linear_slide;

    DcMotor right_front, right_back, left_front, left_back;    //Drive Motors
    DcMotor verticalLeft, verticalRight, horizontal;           //Odometry Wheels

    final double COUNTS_PER_INCH = 307.699557;

    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    //OdometryGlobalCoordinatePosition globalPositionUpdate;


    DigitalChannel frontTouch;


    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    @Override
    public void runOpMode() {


        /*
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

         */


        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        frontTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontTouch.setMode(DigitalChannel.Mode.INPUT);


        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addLine("Gyro: Calibrating");
            telemetry.update();
        }
        telemetry.addLine("Gyro: Done calibrating");
        telemetry.update();

        frontClaw = hardwareMap.get(Servo.class, "frontclaw");
        odometryServo = hardwareMap.get(Servo.class, "odometryServo");
        capstone = hardwareMap.get(Servo.class, "capstone");




        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");
        linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //rightSideDist = hardwareMap.get(DistanceSensor.class, "rightSideDist");
        //leftSideDist = hardwareMap.get(DistanceSensor.class, "leftSideDist");
        //frontLeftDist = hardwareMap.get(DistanceSensor.class, "frontLeftDist");
        //frontRightDist = hardwareMap.get(DistanceSensor.class, "frontRightDist");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode(true);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }



        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        waitForStart();

        odometryServo.setPosition(.5);










        //make sure the claw and foundation Clamp is up
        frontClaw.setPosition(.7);


        //strafeEncoder(4.5,.4);
        moveDistanceEncoder(12, .4);

        capstone.setPosition(.87);




        if (!scanStoneVuforia(targetsSkyStone, allTrackables)) {
            strafeEncoder(6, .3);
            if (!scanStoneVuforia(targetsSkyStone, allTrackables)) {
                pos = "C";
            } else {
                pos = "B";
            }
        } else {
            pos = "A";
        }

        CameraDevice.getInstance().setFlashTorchMode(false);
        targetsSkyStone.deactivate();



        /*
        if (tfod != null) {
            CameraDevice.getInstance().setFlashTorchMode(false);
            tfod.shutdown();
            sleep(500);
        }

         */


        telemetry.addData("Pattern", pos);
        telemetry.update();




        /*
        Stone 1
         */

        //Move close to the stone line
        moveDistanceEncoder(9, .5);


        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(350);
        flipBackRight.setTargetPosition(350);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);





        if (pos.equals("A")) {
            strafeEncoder(-6, .5);
        } else if (pos.equals("B")) {
            strafeEncoder(-3, .5);
        } else if (pos.equals("C")) {
            strafeEncoder(4, .5);
        }



        //Move a little more forward to get close enough to the stone to grab it
        moveDistanceEncoder(3, .25);

        //Close the claw and wait for it to reach the closed position
        frontClaw.setPosition(0);
        sleep(550);

        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);


        moveDistanceEncoder(-2, .85);//back up from stones
        turnDegree(90, .5, 5);  //turn left 90 to drive
        strafeEncoder(2, .5);  //strafe back to the stones





        if (pos.equals("A")) {
            moveDistanceEncoder(76, .85); //move to the other side (middle of the platform)
        } else if (pos.equals("B")) {
            moveDistanceEncoder(83, .85); //move to the other side (middle of the platform)
        } else if (pos.equals("C")) {
            moveDistanceEncoder(92, .85); //move to the other side (middle of the platform)
        }


        turnDegree(-88, .7, 5);  //turn 90 right to drop on platform




        //flip the linear slide down
        flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipBackLeft.setTargetPosition(300);
        flipBackRight.setTargetPosition(300);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setPower(.9);
        flipBackRight.setPower(.9);

        linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_slide.setTargetPosition(500);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (linear_slide.isBusy()) {
            linear_slide.setPower(.7);
        }
        linear_slide.setPower(0);


        //used to be 600
        sleep(200);



        driveAll(0);



        moveDistanceEncoder(-5, .8); //back up a little with the foundation
        foundationMovement(30, 1, -.1);  //rotate the foundation a little
        moveDistanceEncoder(-3, .8);  //back up
        foundationMovement(60, 1, .2);  //rotate the foundation the rest of the way


        flipBackRight.setPower(0);
        flipBackLeft.setPower(0);

        //open claw
        frontClaw.setPosition(1);
        sleep(300);

        linear_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_slide.setTargetPosition(0);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_slide.setPower(.7);

        sleep(200);


        //Flip the linear slide back into the robot
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }
        flipBackLeft.setPower(0);
        flipBackRight.setPower(0);


        if (!linear_slide.isBusy()) {
            linear_slide.setPower(0);
        }


        //push the foundation into the zone
        ElapsedTime moveTime = new ElapsedTime();
        while (opModeIsActive() && moveTime.seconds() < 1.5) {
            driveAll(.3);
        }
        driveAll(0);

        strafeEncoder(15, .5);  //strafe to move under the bridge
        moveDistanceEncoder(-15, .8);   //move under the bridge

        //if the stone was in the first spot, go for the second one
        if (!pos.equals("A")) {
            //flip the linear slide down
            flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackLeft.setTargetPosition(250);
            flipBackRight.setTargetPosition(250);
            flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && flipBackLeft.isBusy()) {
                flipBackLeft.setPower(.6);
                flipBackRight.setPower(.6);
            }
            flipBackLeft.setPower(0);
            flipBackRight.setPower(0);

            moveDistanceEncoder(-19, .8);
        } else {


            moveDistanceEncoder(-50, .8);

            turnDegree(-90, .7, 5);

            //flip the linear slide down
            flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackLeft.setTargetPosition(350);
            flipBackRight.setTargetPosition(350);
            flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && flipBackLeft.isBusy()) {
                flipBackLeft.setPower(.4);
                flipBackRight.setPower(.4);
            }
            flipBackLeft.setPower(0);
            flipBackRight.setPower(0);

            moveDistanceEncoder(9, .5);


            //Close the claw and wait for it to reach the closed position
            frontClaw.setPosition(0);
            sleep(550);

            //Flip the linear slide back into the robot
            flipBackLeft.setTargetPosition(0);
            flipBackRight.setTargetPosition(0);
            flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && flipBackLeft.isBusy()) {
                flipBackLeft.setPower(.4);
                flipBackRight.setPower(.4);
            }
            flipBackLeft.setPower(0);
            flipBackRight.setPower(0);

            moveDistanceEncoder(-5, .5);  //back up some

            turnDegree(90, .7, 5);  //turn to face the bridge

            moveDistanceEncoder(30, .8); //move to the other side

            //flip the linear slide down
            flipBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipBackLeft.setTargetPosition(350);
            flipBackRight.setTargetPosition(350);
            flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && flipBackLeft.isBusy()) {
                flipBackLeft.setPower(.4);
                flipBackRight.setPower(.4);
            }
            flipBackLeft.setPower(0);
            flipBackRight.setPower(0);


            //Close the claw and wait for it to reach the closed position
            frontClaw.setPosition(1);
            sleep(550);

            moveDistanceEncoder(-8, .8);  //move back to park
        }

    }







    /*-------------------------------------------------
     *
     * Methods
     *
     * --------------------------------------------------*/


    public void moveDistanceEncoder(double inches, double speed) {

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks = inches * COUNTS_PER_INCH;

        double targetLeft = verticalLeft.getCurrentPosition() + ticks;
        double targetRight = verticalRight.getCurrentPosition() + ticks;

        double leftPower = speed;
        double rightPower = speed;
        double adjust = 0;

        if (inches < 0) {
            leftPower *= -1;
            rightPower *= -1;
        }

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        while (opModeIsActive() && ((Math.abs(verticalRight.getCurrentPosition() - targetRight) >= 100) &&
                (Math.abs(verticalLeft.getCurrentPosition() - targetLeft) >= 100))) {


            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }


            if (inches > 0 && ((verticalRight.getCurrentPosition() >= (2 * (ticks / 3))) ||
                    (verticalLeft.getCurrentPosition() >= (2 * (ticks / 3))))) {
                leftPower = .25;
                rightPower = .25;
            } else if (inches < 0 && ((verticalRight.getCurrentPosition() <= (2 * (ticks / 3))) ||
                    (verticalLeft.getCurrentPosition() <= (2 * (ticks / 3))))) {
                leftPower = -.25;
                rightPower = -.25;
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(leftPower + adjust);
            right_front.setPower(rightPower - adjust);
            left_back.setPower(leftPower + adjust);
            right_back.setPower(rightPower - adjust);


            if (inches > 0 && ((verticalLeft.getCurrentPosition() > targetLeft) || (verticalRight.getCurrentPosition() > targetRight))) {
                break;
            } else if (inches < 0 && ((verticalLeft.getCurrentPosition() < targetLeft) || (verticalRight.getCurrentPosition() < targetRight))) {
                break;
            }
        }
        driveAll(0);


    }

    public void strafeEncoder(double inches, double speed) {
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks = inches * COUNTS_PER_INCH;

        double targetHorizontal = horizontal.getCurrentPosition() + ticks;

        double power = speed;

        if (inches < 0) {
            power *= -1;
        }
        double adjust = 0;

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        while (opModeIsActive() && (Math.abs(horizontal.getCurrentPosition() - targetHorizontal) >= 100)) {

            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            if (integratedZ > 0) {
                adjust = .04;
            } else if (integratedZ < 0) {
                adjust = -.04;
            }


            if (inches > 0 && horizontal.getCurrentPosition() >= (2 * (ticks / 3))) {
                power = .25;
            } else if (inches < 0 && horizontal.getCurrentPosition() <= (2 * (ticks / 3))) {
                power = -.25;
            } else if (inches > 0 && horizontal.getCurrentPosition() > ticks) {
                break;
            } else if (inches < 0 && horizontal.getCurrentPosition() < ticks) {
                break;
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(power + adjust);
            right_front.setPower(-power - adjust);
            left_back.setPower(-power + adjust);
            right_back.setPower(power - adjust);
        }
        driveAll(0);


    }

    public void turnDegree(double degrees, double speed, double allowedError) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        //double heading = modernRoboticsI2cGyro.getHeading();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        //right is positive

        //modernRoboticsI2cGyro.resetZAxisIntegrator();

        double leftSpeed = speed;
        double rightSpeed = -speed;

        if (degrees > 0) {
            leftSpeed *= -1;
            rightSpeed *= -1;
        }

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && Math.abs(integratedZ - degrees) >= allowedError) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
            telemetry.addData("angle", integratedZ);
            telemetry.addData("speed", leftSpeed);
            telemetry.addData("degrees", degrees);
            telemetry.update();


            if (Math.abs(integratedZ) > Math.abs((degrees * 0.7))) {
                leftSpeed = -.2 * Math.signum(degrees);
                rightSpeed = .2 * Math.signum(degrees);
            } else if (degrees > 0 && integratedZ > degrees) {
                break;
            } else if (degrees < 0 && integratedZ < degrees) {
                break;
            } else {
                leftSpeed = speed * -1 * Math.signum(degrees);
                rightSpeed = speed * Math.signum(degrees);
            }
            driveEach(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

        }
        driveAll(0);

    }

    public void foundationMovement(double rotation, double speed, double assistSpeed) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        double integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
        //right is positive

        //modernRoboticsI2cGyro.resetZAxisIntegrator();

        double leftSpeed = -speed;


        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && Math.abs(integratedZ - rotation) >= 2) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();


            if (integratedZ > rotation / 2) {
                leftSpeed = -.42;
            }
            driveEach(leftSpeed, leftSpeed, assistSpeed, assistSpeed);

            if (integratedZ > rotation) {
                break;
            }

        }
        driveAll(0);
    }


    public void driveEach(double lf, double lb, double rf, double rb) {
        left_front.setPower(lf);
        left_back.setPower(lb);

        right_front.setPower(rf);
        right_back.setPower(rb);
    }

    public void driveAll(double power) {
        left_front.setPower(power);
        left_back.setPower(power);

        right_front.setPower(power);
        right_back.setPower(power);
    }


    public boolean scanStoneVuforia(VuforiaTrackables targetsSkyStone, List<VuforiaTrackable> allTrackables) {
        boolean stoneDetected = false;

        targetsSkyStone.activate();
        ElapsedTime timeToScan = new ElapsedTime();
        while (!isStopRequested() && !stoneDetected && timeToScan.seconds() < 2.4) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if (trackable.getName().equals("Stone Target")) {
                        stoneDetected = true;
                    }
                }
            }

        }
        if (stoneDetected) {
            return true;
        } else {
            return false;
        }

    }


    /*
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


     */


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

        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
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


    /*
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    */

}
