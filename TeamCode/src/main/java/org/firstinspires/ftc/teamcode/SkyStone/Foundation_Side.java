//Copyright (c) 2019 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode.SkyStone;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.File;


@Autonomous(name = "Foundation Side")
public class Foundation_Side extends LinearOpMode {

    Servo frontClaw;
    DcMotor flipBackLeft;
    DcMotor flipBackRight;

    Servo foundationClampLeft;
    Servo foundationClampRight;
    DigitalChannel frontTouch;



    DcMotor right_front, right_back, left_front, left_back;    //Drive Motors
    DcMotor verticalLeft, verticalRight, horizontal;           //Odometry Wheels

    final double COUNTS_PER_INCH = 307.699557;

    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    //OdometryGlobalCoordinatePosition globalPositionUpdate;


    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    private String configuration = "Not Decided";
    private boolean stonesLeft = true;

    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    @Override
    public void runOpMode() {

        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addLine("Gyro: Calibrating");
            telemetry.update();
        }
        telemetry.addLine("Gyro: Done calibrating");
        telemetry.update();

        frontClaw = hardwareMap.get(Servo.class, "frontclaw");

        flipBackLeft = hardwareMap.get(DcMotor.class, "left_flip");
        flipBackRight = hardwareMap.get(DcMotor.class, "right_flip");

        flipBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        foundationClampLeft = hardwareMap.get(Servo.class, "foundationClampLeft");
        foundationClampRight = hardwareMap.get(Servo.class, "foundationClampRight");

        frontTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontTouch.setMode(DigitalChannel.Mode.INPUT);



        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        waitForStart();

        foundationClampLeft.setPosition(.85);
        foundationClampRight.setPosition(.6);

        //moveDistanceEncoder(-7,.5);



        //strafe to align with the foundation
        strafeEncoder(-18, .5);



        //wait 20 seconds before moving the foundation. (Changes per alliance)
        //sleep(20000);



        //Move forward to grab the foundation
        //moveDistanceEncoder(26, .7);
        //move until button pressed
        ElapsedTime allowedMoveTime = new ElapsedTime();
        while (opModeIsActive() && frontTouch.getState() && allowedMoveTime.seconds() < 5) {
            driveAll(.25);
        }
        driveAll(.13);


        foundationClampLeft.setPosition(.7);
        foundationClampRight.setPosition(1);
        sleep(300);

        driveAll(0);


        //Flip the slide forward to grab the foundation
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

        //Back up to the wall with the foundation
        moveDistanceEncoder(-28, .5);


        //Flip the slide back to let go of the foundation
        flipBackLeft.setTargetPosition(0);
        flipBackRight.setTargetPosition(0);
        flipBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && flipBackLeft.isBusy()) {
            flipBackLeft.setPower(.4);
            flipBackRight.setPower(.4);
        }


        //Strafe under the bridge
        strafeEncoder(44, .5);



















        //drive to where you can see the stones
        //moveDistancePID(.5,20);
        //goToPosition(1,1,.5,0,.5);

        //scan the stones
        //scanCV();


        //collect the stones
        /*
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

         */
        //park on middle line
        //drive from the homing position


        //globalPositionUpdate.stop();
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
            }

            // driveEach(leftPower, leftPower, rightPower, rightPower);
            left_front.setPower(power + adjust);
            right_front.setPower(-power - adjust);
            left_back.setPower(-power + adjust);
            right_back.setPower(power - adjust);
        }
        driveAll(0);


    }

    public void turnDegree(double degrees, double speed) {
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

        while (opModeIsActive() && Math.abs(integratedZ - degrees) >= 5) {
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            telemetry.addData("angle", integratedZ);
            telemetry.addData("targ", degrees);
            telemetry.update();

            if (degrees > 0 && integratedZ > degrees / 2) {
                leftSpeed = -.25;
                rightSpeed = .25;
            } else if (degrees < 0 && integratedZ > degrees / 2) {
                leftSpeed = .25;
                rightSpeed = -.25;
            }
            driveEach(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

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

}
